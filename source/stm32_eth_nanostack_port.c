/*
 * Copyright (c) 2014-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if !MBED_CONF_RTOS_PRESENT
#error RTOS is required for `stm32_eth_nanostack_port.c`!
#endif

#define HAVE_DEBUG 1

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "arm_hal_phy.h"
#include "arm_hal_interrupt.h"
#include "ns_types.h"
#include "stm32_eth_nanostack_port.h"
#include "eventOS_event_timer.h"
#include "nsdynmemLIB.h"
#include "cmsis_os.h"
#include "mbed_trace.h"
#include "mbed_error.h"
#include "mbed_debug.h"
#include "mbed_interface.h"

/* Macro Definitions */
#define TRACE_GROUP  "Eth"
#define ENET_ARCH_PHY_ADDRESS         (0x00)
#define ENET_HDR_LEN                  (14)
#if !defined(STM32_NS_ENET_RX_BUF_NB)
#define STM32_NS_ENET_RX_BUF_NB       (6)
#endif
#if !defined(STM32_NS_ENET_TX_BUF_NB)
#define STM32_NS_ENET_TX_BUF_NB       (4)
#endif

/* Signals for IRQ thread */
#define SIG_RX  1

/* This routine starts a 'Thread' which handles IRQs*/
static void Eth_IRQ_Thread_Create(void);

/* Function Prototypes*/
static int8_t stm32_eth_phy_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static int8_t stm32_eth_phy_interface_state_control(phy_interface_state_e state, uint8_t x);
static int8_t stm32_eth_phy_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow);
static void PHY_LinkStatus_Task(void *y);
static void eth_if_lock(void);
static void eth_if_unlock(void);
static int8_t stm32_eth_send(uint8_t *data_ptr, uint16_t data_len);
__weak uint8_t mbed_otp_mac_address(char *mac);

/* Callback function to notify stack about the readiness of the Eth module */
static void (*driver_readiness_status_callback)(uint8_t, int8_t) = 0;

/* Nanostack generic PHY driver structure */
static phy_device_driver_s eth_device_driver;
static ETH_HandleTypeDef EthHandle;

/* Status variables */
static uint8_t eth_driver_enabled = 0;
static int8_t eth_interface_id = -1;
static bool link_currently_up = false;

/* Thread ID for the thread we will start */
static osThreadId eth_irq_thread_id;

/* DMA descriptors & buffers */
#if defined (__ICCARM__)   /*!< IAR Compiler */
#pragma data_alignment=4
#endif
static __ALIGN_BEGIN ETH_DMADescTypeDef DMARxDscrTab[STM32_NS_ENET_RX_BUF_NB] __ALIGN_END; /* Ethernet Rx DMA Descriptor */

#if defined (__ICCARM__)   /*!< IAR Compiler */
#pragma data_alignment=4
#endif
static __ALIGN_BEGIN ETH_DMADescTypeDef DMATxDscrTab[STM32_NS_ENET_TX_BUF_NB] __ALIGN_END; /* Ethernet Tx DMA Descriptor */

#if defined (__ICCARM__)   /*!< IAR Compiler */
#pragma data_alignment=4
#endif
static __ALIGN_BEGIN uint8_t Rx_Buff[STM32_NS_ENET_RX_BUF_NB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */

#if defined (__ICCARM__)   /*!< IAR Compiler */
#pragma data_alignment=4
#endif
static __ALIGN_BEGIN uint8_t Tx_Buff[STM32_NS_ENET_TX_BUF_NB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */


/** This returns a unique 6-byte MAC address, based on the unique device ID register
 *  @param mac A 6-byte array to write the MAC address
 */
static void stm32_default_mac_address(char *mac) {
    unsigned char ST_mac_addr[3] = {0x00, 0x80, 0xe1}; // default STMicro mac address

    // Read unique id
#if defined (TARGET_STM32F2)
    uint32_t word0 = *(uint32_t *)0x1FFF7A10;
#elif defined (TARGET_STM32F4)
    uint32_t word0 = *(uint32_t *)0x1FFF7A10;
#elif defined (TARGET_STM32F7)
    uint32_t word0 = *(uint32_t *)0x1FF0F420;
#else
#error MAC address can not be derived from target unique Id
#endif

    mac[0] = ST_mac_addr[0];
    mac[1] = ST_mac_addr[1];
    mac[2] = ST_mac_addr[2];
    mac[3] = (word0 & 0x00ff0000) >> 16;
    mac[4] = (word0 & 0x0000ff00) >> 8;
    mac[5] = (word0 & 0x000000ff);

    return;
}


/** This returns a unique 6-byte MAC address, based on the device UID
 *  This function overrides hal/common/mbed_interface.c function
 *  @param mac A 6-byte array to write the MAC address
 */
void mbed_mac_address(char *mac) {
    if (mbed_otp_mac_address(mac)) {
        return;
    } else {
        stm32_default_mac_address(mac);
    }
    return;
}

__weak uint8_t mbed_otp_mac_address(char *mac) {
    return 0;
}


/**
 * Ethernet Rx Transfer completed callback
 *
 * @param  heth: ETH handle
 * @retval None
 *
 * @note   Called by 'HAL_ETH_IRQHandler()'
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    osSignalSet(eth_irq_thread_id, SIG_RX);
}


/**
 * Ethernet IRQ Handler
 *
 * @param  None
 * @retval None
 *
 * @note   Set in startup file vector table (i.e. no need to install it by 'NVIC_SetVector()')
 */
void ETH_IRQHandler(void)
{
    HAL_ETH_IRQHandler(&EthHandle);
}


/**
 * In this function, the hardware should be initialized.
 * Called from eth_arch_enetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static const ETH_MACInitTypeDef MacInitConf = {
        .Watchdog = ETH_WATCHDOG_ENABLE,
        .Jabber = ETH_JABBER_ENABLE,
        .InterFrameGap = ETH_INTERFRAMEGAP_96BIT,
        .CarrierSense = ETH_CARRIERSENCE_ENABLE,
        .ReceiveOwn = ETH_RECEIVEOWN_ENABLE,
        .LoopbackMode = ETH_LOOPBACKMODE_DISABLE,
        .ChecksumOffload = ETH_CHECKSUMOFFLAOD_ENABLE,
        .RetryTransmission = ETH_RETRYTRANSMISSION_DISABLE,
        .AutomaticPadCRCStrip = ETH_AUTOMATICPADCRCSTRIP_DISABLE,
        .BackOffLimit = ETH_BACKOFFLIMIT_10,
        .DeferralCheck = ETH_DEFFERRALCHECK_DISABLE,
        .ReceiveAll = ETH_RECEIVEAll_DISABLE,
        .SourceAddrFilter = ETH_SOURCEADDRFILTER_DISABLE,
        .PassControlFrames = ETH_PASSCONTROLFRAMES_BLOCKALL,
        .BroadcastFramesReception = ETH_BROADCASTFRAMESRECEPTION_ENABLE,
        .DestinationAddrFilter = ETH_DESTINATIONADDRFILTER_NORMAL,
        .PromiscuousMode = ETH_PROMISCUOUS_MODE_DISABLE,
        .MulticastFramesFilter = ETH_MULTICASTFRAMESFILTER_NONE,
        .UnicastFramesFilter = ETH_UNICASTFRAMESFILTER_PERFECT,
        .HashTableHigh = 0x0U,
        .HashTableLow = 0x0U,
        .PauseTime = 0x0U,
        .ZeroQuantaPause = ETH_ZEROQUANTAPAUSE_DISABLE,
        .PauseLowThreshold = ETH_PAUSELOWTHRESHOLD_MINUS4,
        .UnicastPauseFrameDetect = ETH_UNICASTPAUSEFRAMEDETECT_DISABLE,
        .ReceiveFlowControl = ETH_RECEIVEFLOWCONTROL_DISABLE,
        .TransmitFlowControl = ETH_TRANSMITFLOWCONTROL_DISABLE,
        .VLANTagComparison = ETH_VLANTAGCOMPARISON_16BIT,
        .VLANTagIdentifier = 0x0U,
};

static void stm32_eth_arch_low_level_init(void)
{
    HAL_StatusTypeDef hal_eth_init_status;

    /* Init ETH */
    EthHandle.Instance = ETH;
    EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    EthHandle.Init.Speed = ETH_SPEED_100M;
    EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    EthHandle.Init.PhyAddress = ENET_ARCH_PHY_ADDRESS;
    EthHandle.Init.MACAddr = &eth_device_driver.PHY_MAC[0]; // WAS: &MACAddr[0];
    EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
    hal_eth_init_status = HAL_ETH_Init(&EthHandle);
    if(hal_eth_init_status != HAL_OK) {
        tr_warn("Failed to initialize Ethernet. (%s, %d, %d)", __func__, __LINE__, hal_eth_init_status);
    }

    hal_eth_init_status = HAL_ETH_ConfigMAC(&EthHandle, (ETH_MACInitTypeDef*)&MacInitConf);
    if(hal_eth_init_status != HAL_OK) {
        tr_warn("Failed to initialize Ethernet. (%s, %d)", __func__, __LINE__);
    }

    /* Initialize Tx Descriptors list: Chain Mode */
    HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], STM32_NS_ENET_TX_BUF_NB);

    /* Initialize Rx Descriptors list: Chain Mode  */
    HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], STM32_NS_ENET_RX_BUF_NB);

    /* Enable MAC and DMA transmission and reception */
    HAL_ETH_Start(&EthHandle);
}


/**
 * This function should do the actual transmission of the packet.
 *
 * @note This function is NOT thread-safe
 */
static int stm32_eth_arch_low_level_output(uint8_t *payload, int len)
{
    int errval;
    uint8_t *buffer = (uint8_t*)(EthHandle.TxDesc->Buffer1Addr);
    __IO ETH_DMADescTypeDef *DmaTxDesc;
    uint32_t framelength = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;
    DmaTxDesc = EthHandle.TxDesc;

    /* copy payload to driver buffers */
    /* Is this buffer available? If not, goto error */
    if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET) {
        errval = -1;
        goto error;
    }

    /* Get bytes in current lwIP buffer */
    byteslefttocopy = len;
    payloadoffset = 0;

    /* Check if the length of data to copy is bigger than Tx buffer size*/
    while (byteslefttocopy > ETH_TX_BUF_SIZE) {
        /* Copy data to Tx buffer*/
        memcpy((uint8_t*)buffer, (uint8_t*)((uint8_t*)payload + payloadoffset), ETH_TX_BUF_SIZE);

        /* Point to next descriptor */
        DmaTxDesc = (ETH_DMADescTypeDef*)(DmaTxDesc->Buffer2NextDescAddr);

        /* Check if the buffer is available */
        if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET) {
            errval = -1;
            goto error;
        }

        buffer = (uint8_t*)(DmaTxDesc->Buffer1Addr);

        byteslefttocopy = byteslefttocopy - ETH_TX_BUF_SIZE;
        payloadoffset = payloadoffset + ETH_TX_BUF_SIZE;
        framelength = framelength + ETH_TX_BUF_SIZE;
    }

    /* Copy the remaining bytes */
    memcpy((uint8_t*)buffer, (uint8_t*)((uint8_t*)payload + payloadoffset), byteslefttocopy);
    framelength = framelength + byteslefttocopy;

    tr_debug("%s (%d): %d", __func__, __LINE__, (int)framelength);

    /* Prepare transmit descriptors to give to DMA */
    HAL_ETH_TransmitFrame(&EthHandle, framelength);

    errval = 0;

error:

    /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
    if ((EthHandle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET) {
        tr_debug("%s (%d)", __func__, __LINE__);

        /* Clear TUS ETHERNET DMA flag */
        EthHandle.Instance->DMASR = ETH_DMASR_TUS;

        /* Resume DMA transmission*/
        EthHandle.Instance->DMATPDR = 0;
    }

    tr_debug("%s (%d): %d", __func__, __LINE__, errval);
    return errval;
}


/**
 * Helper function for `stm32_eth_arch_low_level_input()`
 */
static inline void nanostack_rx_call(const uint8_t *data_ptr, uint16_t data_len) {
    // call Nanostack callback
    if (eth_device_driver.phy_rx_cb) {
        tr_debug("%s (%d): len=%d", __func__, __LINE__, data_len);
        eth_device_driver.phy_rx_cb(data_ptr, data_len, 0xff, 0, eth_interface_id);
    }
}

/**
 * Should transfer the bytes of the incoming packet to the buffer passed as param.
 *
 * @note This function is NOT thread-safe
 */
static int stm32_eth_arch_low_level_input()
{
    uint16_t len = 0;
    uint8_t *buffer;
    __IO ETH_DMADescTypeDef *dmarxdesc;
    uint32_t payloadoffset = 0;
    uint32_t byteslefttocopy = 0;

    /* get received frame */
    if (HAL_ETH_GetReceivedFrame(&EthHandle) != HAL_OK) {
        tr_debug("%s (%d)", __func__, __LINE__);
        return -1;
    }

    /* Obtain the size of the packet and put it into the "len" variable. */
    len = EthHandle.RxFrameInfos.length;
    buffer = (uint8_t*)EthHandle.RxFrameInfos.buffer;

    byteslefttocopy = len;
    payloadoffset = 0;

    dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
    {
        /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size*/
        while (byteslefttocopy > ETH_RX_BUF_SIZE) {
            /* Call nanostack */
            nanostack_rx_call((const uint8_t*)buffer, ETH_RX_BUF_SIZE);

            /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
            dmarxdesc->Status |= ETH_DMARXDESC_OWN;

            /* Point to next descriptor */
            dmarxdesc = (ETH_DMADescTypeDef*)(dmarxdesc->Buffer2NextDescAddr);
            buffer = (uint8_t*)(dmarxdesc->Buffer1Addr);

            byteslefttocopy = byteslefttocopy - ETH_RX_BUF_SIZE;
            payloadoffset = payloadoffset + ETH_RX_BUF_SIZE;
        }

        /* Call nanostack with remaining data */
        nanostack_rx_call((const uint8_t*)buffer, byteslefttocopy);
        byteslefttocopy = 0;

        /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
        dmarxdesc->Status |= ETH_DMARXDESC_OWN;
    }

    /* Clear Segment_Count */
    EthHandle.RxFrameInfos.SegCount = 0;

    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
    if ((EthHandle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET) {
        /* Clear RBUS ETHERNET DMA flag */
        EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        EthHandle.Instance->DMARPDR = 0;
    }

    tr_debug("%s (%d): %d", __func__, __LINE__, (int)(len - byteslefttocopy));

    return len - byteslefttocopy;
}


static void stm32_eth_arch_enable_interrupts(void)
{
    tr_debug("%s (%d)", __func__, __LINE__);

    HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}


/** \brief  Function to register the ethernet driver to the Nanostack
 *
 *  \param[in] mac_ptr  Pointer to MAC address
 *  \param[in] driver_status_cb  Function pointer to notify the caller about driver status
 */
void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*driver_status_cb)(uint8_t, int8_t))
{
    if (eth_interface_id < 0) {

        eth_device_driver.PHY_MAC = mac_ptr;
        eth_device_driver.address_write = &stm32_eth_phy_address_write;
        eth_device_driver.driver_description = "ETH";
        eth_device_driver.link_type = PHY_LINK_ETHERNET_TYPE;
        eth_device_driver.phy_MTU = ETH_RX_BUF_SIZE;
        eth_device_driver.phy_header_length = 0;
        eth_device_driver.phy_tail_length = 0;
        eth_device_driver.state_control = &stm32_eth_phy_interface_state_control;
        eth_device_driver.tx = &stm32_eth_phy_tx;
        eth_device_driver.phy_rx_cb = NULL;
        eth_device_driver.phy_tx_done_cb = NULL;
        eth_interface_id = arm_net_phy_register(&eth_device_driver);
        driver_readiness_status_callback = driver_status_cb;

        if (eth_interface_id < 0){
            tr_error("Ethernet Driver failed to register with Stack. RetCode=%i", eth_driver_enabled);
            driver_readiness_status_callback(0, eth_interface_id);
            return;
        }
    }

    if (!eth_driver_enabled) {
        stm32_eth_arch_low_level_init();
#if !DEVICE_EMAC
        stm32_eth_arch_enable_interrupts();
#endif
        eth_driver_enabled = 1;
        driver_readiness_status_callback(link_currently_up, eth_interface_id);
        Eth_IRQ_Thread_Create();
        eventOS_timeout_ms(PHY_LinkStatus_Task, 500, NULL);
    }
}


/** \brief  TX routine used by Nanostack to transmit via Ethernet Interface
 *
 *  \param[in] data_ptr   Pointer to data packet
 *  \param[in] drta_len   Length of the data packet
 *  \param[in] tx_handle  Not used in this context. Safe to pass Null.
 *  \param[in] tdata_flow Not used in this context. Safe to pass Null.
 */
static int8_t stm32_eth_phy_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow)
{
    int retval = -1;

    if(data_len >= ENET_HDR_LEN){
        retval = stm32_eth_send(data_ptr, data_len);
    }

    (void)data_flow;
    (void)tx_handle;

    return retval;
}


/* TODO State Control Handling.*/
static int8_t stm32_eth_phy_interface_state_control(phy_interface_state_e state, uint8_t not_required)
{
    switch(state){
        case PHY_INTERFACE_DOWN:
            break;
        case PHY_INTERFACE_UP:
            break;
        case PHY_INTERFACE_RESET:
            break;
        case PHY_INTERFACE_SNIFFER_STATE:
            /*TODO Allow promiscuous state here*/
            break;
        case PHY_INTERFACE_RX_ENERGY_STATE:
            /*Just to get rid of compiler warning*/
            break;

    }

    (void)not_required;

    return 0;
}


/** \brief  Function to send data packets
 *
 * This function is called by stm32_eth_phy_tx() which is in turn called through
 * Nanostack.
 *
 *  \param[in] data_ptr  Pointer to the data buffer
 *  \param[in] data_len  Length of the data
 *
 *  \returns 0 if successful, -1 if unsuccessful
 */
static int8_t stm32_eth_send(uint8_t *data_ptr, uint16_t data_len)
{
    int ret;

    // Get lock
    eth_if_lock();

    /* Setup transfer */
    ret = stm32_eth_arch_low_level_output(data_ptr, data_len);

    // Release lock
    eth_if_unlock();

    return ret;
}


/**
 * Helper function for `stm32_eth_phy_address_write()`
 */
static inline void stm32_set_mac_48bit(uint8_t *ptr) {
    tr_debug("%s (%d), adr0=%x, adr1=%x, adr2=%x, adr3=%x, adr4=%x, adr5=%x",
             __func__, __LINE__,
             ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
}

/** \brief  Stack sets the MAC address using this routine
 *
 * This function sets the MAC address and its type for the Ethernet interface
 * Only type supported is 48 bits.
 *
 *  \param[in] address_type   Type of MAC address, i.e., 48 bit, 64 bit etc.
 *  \param[in] address_ptr    Pointer to MAC address
 *
 *  \return  0 if successful <0 if unsuccessful
 */
static int8_t stm32_eth_phy_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    int8_t retval = 0;

    switch(address_type){
        case PHY_MAC_48BIT:
            stm32_set_mac_48bit(address_ptr); /* betzw - WAS: stm32_eth_set_address(address_ptr);
        	                                     But once initialized, we cannot change MAC address in the driver!?!
             */
            break;
        case PHY_MAC_64BIT:
        case PHY_MAC_16BIT:
        case PHY_MAC_PANID:
            retval=-1;
            break;
    }

    return retval;
}


/**
 * Helper function for `PHY_LinkStatus_Task()`
 */
static inline bool stm32_get_link_status() {
    uint32_t status;
    bool ret = false;

    if (HAL_ETH_ReadPHYRegister(&EthHandle, PHY_BSR, &status) == HAL_OK) {
        if (status & PHY_LINKED_STATUS) {
            ret = true;
        } else if (!(status & PHY_LINKED_STATUS)) {
            ret = false;
        }
    }

    return ret;
}

/** \brief  Task check the status of PHY link
 *
 *      This task PHY link status and tells the stack if the Eth cable is
 *      connected or not. Checks the status after every 500 millisecond.
 *
 *  \param[in] Optional user-given parameter
 */
static void PHY_LinkStatus_Task(void *y)
{
    bool link = false;

    eth_if_lock();
    link = stm32_get_link_status();
    if (link != link_currently_up) {
        link_currently_up = link;
        eth_if_unlock();
        driver_readiness_status_callback(link, eth_interface_id);
        tr_info ("Ethernet cable %s.", link ? "connected" : "unplugged");
    } else {
        eth_if_unlock();
    }

    eventOS_timeout_ms(PHY_LinkStatus_Task, 500, NULL);
}


/** \brief  Function to get a lock on the Thread.
 * An abstraction of platform_enter_critical.
 * In RTOS Mode: It claims a mutex and protects the foreground operations from
 *               background stuff.
 * In Non-RTOS Mode: It disables interrupts and hence protects foreground Ops
 *               from background ones.
 */
static void eth_if_lock(void)
{
    platform_enter_critical();
}


/** \brief  Function to release a lock from the Thread.
 * An abstraction of platform_exit_critical.
 * In RTOS Mode: Makes sure that we have released the mutex.
 * In Non-RTOS Mode: Enables interrupts etc.
 */
static void eth_if_unlock(void)
{
    platform_exit_critical();
}


/** \brief  Interrupt Service Routine for RX IRQ
 *      If Mbed RTOS is being used, this routine will be called from the
 *      Eth_IRQ_Thread (thread context) after receiving the signal from
 *      interrupt context (SIG_RX).
 *      Otherwise, it will be called directly from the interrupt context.
 */
static void enet_rx_task(void)
{
    int ret;

    do {
        ret = stm32_eth_arch_low_level_input();
    } while(ret > 0);
}


/** \brief  Thread started by ETH_IRQ_Thread_Create
 *
 *      Used only in case of  mbed RTOS. This Thread handles the signals coming
 *      from interrupt context.
 *
 *  \param[in] Optional user-given parameter
 */
static void Eth_IRQ_Thread(const void *x)
{
    for (;;) {
        osEvent event = osSignalWait(0, osWaitForever);
        if (event.status != osEventSignal) {
            continue;
        }

        MBED_ASSERT(event.value.signals & SIG_RX);

        // Get lock
        eth_if_lock();

        /* Setup transfer */
        enet_rx_task();

        /* Release lock */
        eth_if_unlock();
    }
}


/** \brief  Function creating the IRQ thread
 *
 *      Used only in case of  mbed RTOS. Creates a thread for IRQ task.
 */
static void Eth_IRQ_Thread_Create(void)
{
    static osThreadDef(Eth_IRQ_Thread, osPriorityRealtime, 512 * 4 /* betzw - WAS: 512 */);
    eth_irq_thread_id = osThreadCreate(osThread(Eth_IRQ_Thread), NULL);
}
