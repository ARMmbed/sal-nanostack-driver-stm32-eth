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


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "arm_hal_phy.h"
#include "arm_hal_interrupt.h"
#include "ns_types.h"
#include "stm32_eth_nanostack_port.h"
#include "eventOS_event_timer.h"
#define HAVE_DEBUG 1
#include "ns_trace.h"
#include "nsdynmemLIB.h"
#ifdef MBED_CONF_RTOS_PRESENT
#include "cmsis_os.h"
#endif
#include "mbed_interface.h"

#define ETH_ARCH_PHY_ADDRESS    (0x00)

/* Macro Definitions */
#ifndef MEM_ALLOC
#define MEM_ALLOC ns_dyn_mem_alloc
#endif
#ifndef MEM_FREE
#define MEM_FREE ns_dyn_mem_free
#endif
#define TRACE_GROUP  "Eth"
#define ENET_HDR_LEN                  (14)
#define ENET_RX_RING_LEN              (4)
#define ENET_TX_RING_LEN              (4)
// TODO: #define ENET_ETH_MAX_FLEN             (1518)
#define ENET_PTR_ALIGN(x,align)       ((void *)(((uintptr_t)(x) + ((align)-1)) & (~(uintptr_t)((align)- 1))))
#define ENET_SIZE_ALIGN(x,align)      (((size_t)(x) + ((align)-1)) & (~(size_t)((align)- 1)))
// TODO: #define ENET_BuffPtrAlign(n)          ENET_PTR_ALIGN(n, ENET_BUFF_ALIGNMENT)
// TODO: #define ENET_BuffSizeAlign(n)         ENET_SIZE_ALIGN(n, ENET_BUFF_ALIGNMENT)


#ifdef MBED_CONF_RTOS_PRESENT
/* Thread IDs for the threads we will start */
static osThreadId eth_irq_thread_id;

/* Signals for IRQ thread */
#define SIG_TX  1
#define SIG_RX  2

/* This routine starts a 'Thread' which handles IRQs*/
static void Eth_IRQ_Thread_Create(void);
#endif /*MBED_CONF_RTOS_PRESENT*/


/* Function Prototypes*/
static int8_t stm32_eth_phy_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
// betzw - NEEDED?!?: static void stm32_eth_set_address(uint8_t *address_ptr);
static int8_t stm32_eth_phy_interface_state_control(phy_interface_state_e state, uint8_t x);
static int8_t stm32_eth_phy_tx(uint8_t *data_ptr, uint16_t data_len, uint8_t tx_handle,data_protocol_e data_flow);
static void PHY_LinkStatus_Task(void *y);
static void eth_if_lock(void);
static void eth_if_unlock(void);
static void stm32_eth_initialize(uint8_t *mac_addr);
static int8_t stm32_eth_send(const uint8_t *data_ptr, uint16_t data_len);
// TODO: static void k64f_eth_receive(volatile enet_rx_bd_struct_t *bdPtr);
static void update_read_buffer(void);
// TODO: static void ethernet_event_callback(ENET_Type *base, enet_handle_t *handle, enet_event_t event, void *param);

/* Callback function to notify stack about the readiness of the Eth module */
void (*driver_readiness_status_callback)(uint8_t, int8_t) = 0;

/* External function from hardware_init.c
 * Initializes the Eth module hardware */
// TODO: extern void initialize_enet_hardware(void);

/* Global Eth module Handle*/
// TODO: static enet_handle_t global_enet_handle;

/* Nanostack generic PHY driver structure */
static phy_device_driver_s eth_device_driver;

static ETH_HandleTypeDef EthHandle;

#if defined (__ICCARM__)   /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
static __ALIGN_BEGIN ETH_DMADescTypeDef DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END; /* Ethernet Rx DMA Descriptor */

#if defined (__ICCARM__)   /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
static __ALIGN_BEGIN ETH_DMADescTypeDef DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END; /* Ethernet Tx DMA Descriptor */

#if defined (__ICCARM__)   /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
static __ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */

#if defined (__ICCARM__)   /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
static __ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */


/* Definitions for error constants. */
/** No error, everything OK. */
#define ERR_OK          0
/** Out of memory error.     */
#define ERR_MEM        -1
/** Buffer error.            */
#define ERR_BUF        -2
/** Timeout.                 */
#define ERR_TIMEOUT    -3
/** Routing problem.         */
#define ERR_RTE        -4
/** Operation in progress    */
#define ERR_INPROGRESS -5
/** Illegal value.           */
#define ERR_VAL        -6
/** Operation would block.   */
#define ERR_WOULDBLOCK -7
/** Address in use.          */
#define ERR_USE        -8
/** Already connecting.      */
#define ERR_ALREADY    -9
/** Conn already established.*/
#define ERR_ISCONN     -10
/** Not connected.           */
#define ERR_CONN       -11
/** Low-level netif error    */
#define ERR_IF         -12

#define ERR_IS_FATAL(e) ((e) <= ERR_ABRT)
/** Connection aborted.      */
#define ERR_ABRT       -13
/** Connection reset.        */
#define ERR_RST        -14
/** Connection closed.       */
#define ERR_CLSD       -15
/** Illegal argument.        */
#define ERR_ARG        -16


/** This returns a unique 6-byte MAC address, based on the unique device ID register
*  @param mac A 6-byte array to write the MAC address
*/
static void stm32_default_mac_address(char *mac) {
    unsigned char ST_mac_addr[3] = {0x00, 0x80, 0xe1}; // default STMicro mac address

    // Read unic id
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
#ifdef MBED_CONF_RTOS_PRESENT
            osSignalSet(eth_irq_thread_id, SIG_RX);
#else
            enet_rx_task();
#endif /*MBED_CONF_RTOS_PRESENT*/
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
static void stm32_eth_arch_low_level_init(void)
{
    uint32_t regvalue = 0;
    HAL_StatusTypeDef hal_eth_init_status;

    /* Init ETH */
    EthHandle.Instance = ETH;
    EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    EthHandle.Init.Speed = ETH_SPEED_100M;
    EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    EthHandle.Init.PhyAddress = ETH_ARCH_PHY_ADDRESS;
    EthHandle.Init.MACAddr = &eth_device_driver.PHY_MAC[0]; // WAS: &MACAddr[0];
    EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
    hal_eth_init_status = HAL_ETH_Init(&EthHandle);

    /* Initialize Tx Descriptors list: Chain Mode */
    HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

    /* Initialize Rx Descriptors list: Chain Mode  */
    HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

    /* Enable MAC and DMA transmission and reception */
    HAL_ETH_Start(&EthHandle);
}


/**
 * This function should do the actual transmission of the packet.
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 *
 * @note This function is NOT thread-safe
 */
static int stm32_eth_arch_low_level_output(uint8_t *payload, int len)
{
    int errval;
    uint8_t *buffer = (uint8_t*)(EthHandle.TxDesc->Buffer1Addr);
    __IO ETH_DMADescTypeDef *DmaTxDesc;
    uint32_t framelength = 0;
    uint32_t bufferoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;
    DmaTxDesc = EthHandle.TxDesc;
    bufferoffset = 0;

    /* copy payload to driver buffers */
    {
        /* Is this buffer available? If not, goto error */
        if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET) {
            errval = ERR_USE;
            goto error;
        }

        /* Get bytes in current lwIP buffer */
        byteslefttocopy = len;
        payloadoffset = 0;

        /* Check if the length of data to copy is bigger than Tx buffer size*/
        while ((byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE) {
            /* Copy data to Tx buffer*/
            memcpy((uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)payload + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset));

            /* Point to next descriptor */
            DmaTxDesc = (ETH_DMADescTypeDef*)(DmaTxDesc->Buffer2NextDescAddr);

            /* Check if the buffer is available */
            if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET) {
                errval = ERR_USE;
                goto error;
            }

            buffer = (uint8_t*)(DmaTxDesc->Buffer1Addr);

            byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
            payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
            framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
            bufferoffset = 0;
        }

        /* Copy the remaining bytes */
        memcpy((uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)payload + payloadoffset), byteslefttocopy);
        bufferoffset = bufferoffset + byteslefttocopy;
        framelength = framelength + byteslefttocopy;
    }

    /* Prepare transmit descriptors to give to DMA */
    HAL_ETH_TransmitFrame(&EthHandle, framelength);

    errval = ERR_OK;

error:

    /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
    if ((EthHandle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET) {
        /* Clear TUS ETHERNET DMA flag */
        EthHandle.Instance->DMASR = ETH_DMASR_TUS;

        /* Resume DMA transmission*/
        EthHandle.Instance->DMATPDR = 0;
    }

    return errval;
}


/**
 * Should transfer the bytes of the incoming packet to the buffer passed as param.
 *
 * @note The passed buffer must be at least of size 'ETH_MAX_ETH_PAYLOAD'
 * @note This function is NOT thread-safe
 */
static int stm32_eth_arch_low_level_input(uint8_t *paramBuffer, int paramLen)
{
    uint16_t len = 0;
    uint8_t *q;
    uint8_t *buffer;
    __IO ETH_DMADescTypeDef *dmarxdesc;
    uint32_t bufferoffset = 0;
    uint32_t payloadoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t i = 0;

    if((paramBuffer == NULL) || (paramLen < ETH_MAX_ETH_PAYLOAD)) {
    	return -1;
    }

    /* get received frame */
    if (HAL_ETH_GetReceivedFrame(&EthHandle) != HAL_OK) {
        return -2;
    }

    /* Obtain the size of the packet and put it into the "len" variable. */
    len = EthHandle.RxFrameInfos.length;
    buffer = (uint8_t*)EthHandle.RxFrameInfos.buffer;

    if(paramLen < len) {
    	error("Should never happen");
    }

    bufferoffset = 0;
    q = paramBuffer;
    byteslefttocopy = paramLen;
    payloadoffset = 0;

    if((paramBuffer != NULL) && (paramLen >= len)) {
        dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
        {
            /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size*/
            while ((byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE) {
                /* Copy data to pbuf */
                memcpy((uint8_t*)((uint8_t*)q + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));

                /* Point to next descriptor */
                dmarxdesc = (ETH_DMADescTypeDef*)(dmarxdesc->Buffer2NextDescAddr);
                buffer = (uint8_t*)(dmarxdesc->Buffer1Addr);

                byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
                payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
                bufferoffset = 0;
            }
            /* Copy remaining data in pbuf */
            memcpy((uint8_t*)((uint8_t*)q + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), byteslefttocopy);
            bufferoffset = bufferoffset + byteslefttocopy;
            byteslefttocopy = 0;
        }

        /* Release descriptors to DMA */
        /* Point to first descriptor */
        dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
        /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
        for (i = 0; i < EthHandle.RxFrameInfos.SegCount; i++) {
            dmarxdesc->Status |= ETH_DMARXDESC_OWN;
            dmarxdesc = (ETH_DMADescTypeDef*)(dmarxdesc->Buffer2NextDescAddr);
        }

        /* Clear Segment_Count */
        EthHandle.RxFrameInfos.SegCount = 0;
    }

    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
    if ((EthHandle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET) {
        /* Clear RBUS ETHERNET DMA flag */
        EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        EthHandle.Instance->DMARPDR = 0;
    }

    return paramLen - byteslefttocopy;
}


static void stm32_eth_arch_enable_interrupts(void)
{
    HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

static void stm32_eth_arch_disable_interrupts(void)
{
    NVIC_DisableIRQ(ETH_IRQn);
}


/* Main data structure for keeping Eth module data */
typedef struct Eth_Buf_Data_S {
    /* Pointers to start addresses of TX/RX Buffer descriptor*/
    // TODO: volatile enet_rx_bd_struct_t *rx_buf_desc_start_addr;
    // TODO: volatile enet_tx_bd_struct_t *tx_buf_desc_start_addr;
    /* Pointer to Buffers themselves */
    uint8_t *rx_buf[ENET_RX_RING_LEN];
    uint8_t *tx_buf[ENET_TX_RING_LEN];
} Eth_Buf_Data_S;

/* Global Variables */
static Eth_Buf_Data_S base_DS;
static uint8_t eth_driver_enabled = 0;
static int8_t eth_interface_id = -1;
static bool link_currently_up = false;


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
        eth_device_driver.phy_MTU = ETH_MAX_ETH_PAYLOAD;
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
        stm32_eth_initialize(mac_ptr);
        eth_driver_enabled = 1;
        driver_readiness_status_callback(link_currently_up, eth_interface_id);
#ifdef MBED_CONF_RTOS_PRESENT
        Eth_IRQ_Thread_Create();
#endif
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


/** \brief  Function to update the RX buffer descriptors
 *
 *  Moves the pointer to next buffer descriptor. If its the end of the Ring, wraps
 *  it around.
 */
static void update_read_buffer(void)
{
#if 0 // TODO
    /* Clears status. */
    global_enet_handle.rxBdCurrent->control &= ENET_BUFFDESCRIPTOR_RX_WRAP_MASK;

    /* Sets the receive buffer descriptor with the empty flag. */
    global_enet_handle.rxBdCurrent->control |= ENET_BUFFDESCRIPTOR_RX_EMPTY_MASK;

    /* Increases the buffer descriptor to the next one. */
    if (global_enet_handle.rxBdCurrent->control & ENET_BUFFDESCRIPTOR_RX_WRAP_MASK) {
        global_enet_handle.rxBdCurrent = global_enet_handle.rxBdBase;
    } else {
        global_enet_handle.rxBdCurrent++;
    }

    /* Actives the receive buffer descriptor. */
    ENET->RDAR = ENET_RDAR_RDAR_MASK;
#endif // 0
}


#if 0 // TODO
/** \brief  Function to receive data packets
 *
 *  Reads the data from the buffer provided by the buffer descriptor and hands
 *  it over to the Nanostack.
 *
 *  \param[in] bdPtr  Pointer to the receive buffer descriptor structure
 *  \param[in] idx    Index of the current buffer descriptor
 */
static void k64f_eth_receive(volatile enet_rx_bd_struct_t *bdPtr)
{
    if (!(bdPtr->control & ENET_BUFFDESCRIPTOR_RX_ERR_MASK)) {
        /* Hand it over to Nanostack*/
        if (eth_device_driver.phy_rx_cb) {
            eth_device_driver.phy_rx_cb(bdPtr->buffer, bdPtr->length, 0xff,
                                                 0, eth_interface_id);
        }
    }
}
#endif // 0


/** \brief  Function to reclaim used TX buffer descriptor
 *
 *  This function is called after a TX interrupt takes place. The interrupt will
 *  kick the IRQ thread which will eventually call this function.
 */
static void tx_queue_reclaim(void)
{
#if 0 // TODO
    // Traverse all descriptors, looking for the ones modified by the uDMA
    while (!(global_enet_handle.txBdDirty->control & ENET_BUFFDESCRIPTOR_TX_READY_MASK) && global_enet_handle.txBdDirty->buffer) {
        uint8_t i = global_enet_handle.txBdDirty - global_enet_handle.txBdBase;
        global_enet_handle.txBdDirty->buffer = NULL;
        MEM_FREE(base_DS.tx_buf[i]);
        base_DS.tx_buf[i] = NULL;

        if (global_enet_handle.txBdDirty->control & ENET_BUFFDESCRIPTOR_TX_WRAP_MASK) {
            global_enet_handle.txBdDirty = global_enet_handle.txBdBase;
        } else {
            global_enet_handle.txBdDirty++;
        }
    }
#endif // 0
}


/** \brief  Function to send data packets
 *
 * This function is called by stm32_eth_phy_tx() which is in turn called through
 * Nanostack.
 *
 *  \param[in] data_ptr  Pointer to the data buffer
 *  \param[in] data_len  Length of the data
 *
 *  \returns 0 if successfull, <0 if unsuccessful
 */
static int8_t stm32_eth_send(const uint8_t *data_ptr, uint16_t data_len)
{
    /*Sanity Check*/
    if (data_len > ETH_MAX_ETH_PAYLOAD) {
        tr_error("Packet size bigger than ETH_MAX_ETH_PAYLOAD.");
        return -1;
    }

#if 0 // TODO
    /* Get the index of the next TX descriptor */
    uint8_t index = global_enet_handle.txBdCurrent - global_enet_handle.txBdBase;

    /* Check if next descriptor is free - reclaim should have freed tx_buf */
    if (base_DS.tx_buf[index]) {
        tr_error("TX buf descriptors full. Can't queue packet.");
        return -1;
    }

    uint8_t *buf_ptr = MEM_ALLOC(data_len + ENET_BUFF_ALIGNMENT);
    if (!buf_ptr) {
        tr_error("Alloc failed");
        return -1;
    }

    // Protect against reclaim routine swallowing tx_buf before we set READY
    eth_if_lock();
    base_DS.tx_buf[index] = buf_ptr;
    uint8_t *aligned_ptr = ENET_BuffPtrAlign(buf_ptr);
    memcpy(aligned_ptr, data_ptr, data_len);

    /* Setup transfers */
    global_enet_handle.txBdCurrent->buffer = aligned_ptr;
    global_enet_handle.txBdCurrent->length = data_len;
    global_enet_handle.txBdCurrent->control |=
            (ENET_BUFFDESCRIPTOR_TX_READY_MASK
                    | ENET_BUFFDESCRIPTOR_TX_LAST_MASK);

    /* Increase the buffer descriptor address. */
    if (global_enet_handle.txBdCurrent->control
            & ENET_BUFFDESCRIPTOR_TX_WRAP_MASK) {
        global_enet_handle.txBdCurrent = global_enet_handle.txBdBase;
    } else {
        global_enet_handle.txBdCurrent++;
    }
    eth_if_unlock();

    /* Active the transmit buffer descriptor. */
    ENET->TDAR = ENET_TDAR_TDAR_MASK;
#endif // 0

    return 0;
}


#if 0 // betzw - NEEDED?!?
/** \brief  Sets up MAC address
 *
 * This function is calle by stm32_eth_phy_address_write() which is in turn
 * called by nanostack in order to set up MAC address
 *
 *  \param[in] address_ptr   Pointer to MAC address block
 */
static void stm32_eth_set_address(uint8_t *address_ptr)
{
#if 0 // TODO
    /* When pointer to the MAC address is given. It could be 48-bit EUI generated
     * from Radio, like atmel RF or manually inserted. Preferred method.*/
    ENET_SetMacAddr(ENET, address_ptr);
#endif // 0
}
#endif // 0


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
        	stm32_set_mac_48bit(address_ptr); // betzw - WAS: stm32_eth_set_address(address_ptr);
            break;
        case PHY_MAC_64BIT:
        case PHY_MAC_16BIT:
        case PHY_MAC_PANID:
            retval=-1;
            break;
    }

    return retval;
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
#if 0 // TODO
    bool link = false;

    eth_if_lock();
    PHY_GetLinkStatus(ENET, 0, &link);
    if (link != link_currently_up) {
        link_currently_up = link;
        if (link) {
            phy_duplex_t phy_duplex;
            phy_speed_t phy_speed;
            PHY_GetLinkSpeedDuplex(ENET, 0, &phy_speed, &phy_duplex);
            /* Poke the registers*/
            ENET_SetMII(ENET, (enet_mii_speed_t)phy_speed, (enet_mii_duplex_t)phy_duplex);
        }
        eth_if_unlock();
        driver_readiness_status_callback(link, eth_interface_id);
        tr_info ("Ethernet cable %s.", link ? "connected" : "unplugged");
    } else {
        eth_if_unlock();
    }
#endif // 0

    eventOS_timeout_ms(PHY_LinkStatus_Task, 500, NULL);
}


/** \brief  Initialization function for Ethernet module.
 *
 *      This function initializes the ethernet module using default
 *      configuration. Link speed and duplex is auto-negotiated. It also sets up
 *      the buffer descriptors. If MAC address is setup later by the stack using
 *      stm32_eth_address_set() routine, that will just overwrite the MAC directly
 *      in Hardware registers.
 *
 *  \param[in] mac_addr Pointer to MAC address.
 *
 */
static void stm32_eth_initialize(uint8_t *mac_addr)
{
#if 0 // TODO
    uint32_t sysClock;
    phy_speed_t phy_speed;
    phy_duplex_t phy_duplex;
    uint32_t phyAddr = 0;

    enet_config_t config;


    /* Allocate TX buffer descriptors */
    void *memptr = MEM_ALLOC(
            sizeof(enet_tx_bd_struct_t) * ENET_TX_RING_LEN + ENET_BUFF_ALIGNMENT);
    if (!memptr) {
        tr_error("Could not allocate memory for TX Buf Descriptors.");
        return;
    }
    base_DS.tx_buf_desc_start_addr = ENET_BuffPtrAlign(memptr);
    memset((enet_tx_bd_struct_t *) base_DS.tx_buf_desc_start_addr, 0,
           sizeof(enet_tx_bd_struct_t) * ENET_TX_RING_LEN);

    /* Allocate RX buffer descriptors */
    memptr = MEM_ALLOC( sizeof(enet_rx_bd_struct_t) * ENET_RX_RING_LEN + ENET_BUFF_ALIGNMENT);
    if (!memptr) {
        tr_error("Could not allocate memory for RX Buf Descriptors");
        return;
    }
    base_DS.rx_buf_desc_start_addr = ENET_BuffPtrAlign(memptr);
    memset((enet_rx_bd_struct_t *) base_DS.rx_buf_desc_start_addr, 0,
           sizeof(enet_rx_bd_struct_t) * ENET_RX_RING_LEN);

    /* Allocate RX buffers in one contiguous block */
    memptr = MEM_ALLOC(ENET_BuffSizeAlign(ENET_ETH_MAX_FLEN) * ENET_RX_RING_LEN + ENET_BUFF_ALIGNMENT);
    if (!memptr) {
        tr_error("Could not allocate memory for RX Buffers");
        return;
    }
    base_DS.rx_buf[0] = ENET_BuffPtrAlign(memptr);

    /* Fill in pointers to following buffers */
    for (int i = 1; i < ENET_RX_RING_LEN; i++) {
        base_DS.rx_buf[i] = base_DS.rx_buf[i-1] + ENET_BuffSizeAlign(ENET_ETH_MAX_FLEN);
    }

    /* Preparing the Buffer Configurations */
    enet_buffer_config_t buf_config = {
        .rxBdNumber = ENET_RX_RING_LEN,
        .txBdNumber = ENET_TX_RING_LEN,
        .rxBuffSizeAlign = ENET_BuffSizeAlign(ENET_ETH_MAX_FLEN),
        .txBuffSizeAlign = 0,
        .rxBdStartAddrAlign = base_DS.rx_buf_desc_start_addr,
        .txBdStartAddrAlign = base_DS.tx_buf_desc_start_addr,
        // it appears this is a wrongly-typed pointer to an array of buffer pointers
        .rxBufferAlign = (uint8_t *) base_DS.rx_buf,
        .txBufferAlign = NULL
    };

    /* Initialize the Ethernet Hardware */
    initialize_enet_hardware();

    /* Setup Clock for ENET module */
    sysClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);

    /* Load the Default config (RMII Mode, Full Duplex, 100 Mbps)*/
    ENET_GetDefaultConfig(&config);

    /* Initialize PHY layer */
    PHY_Init(ENET, phyAddr, sysClock);

    /* Setup PHY layer */
    PHY_GetLinkStatus(ENET, phyAddr, &link_currently_up);
    tr_info("Ethernet cable is %sconnected.", link_currently_up ? "" : "not ");
    if (link_currently_up) {
        /* Get link information from PHY (Auto negotiation, in case deafult
         * config is different from actual link available )  */
        PHY_GetLinkSpeedDuplex(ENET, phyAddr, &phy_speed, &phy_duplex);
        /* Change the MII speed and duplex for actual link status. */
        config.miiSpeed = (enet_mii_speed_t) phy_speed;
        config.miiDuplex = (enet_mii_duplex_t) phy_duplex;
    }
    config.rxMaxFrameLen = ENET_ETH_MAX_FLEN;
    config.macSpecialConfig = kENET_ControlFlowControlEnable;
    config.rxAccelerConfig = kENET_RxAccelMacCheckEnabled;
    config.interrupt = kENET_RxFrameInterrupt | kENET_TxFrameInterrupt;

    /* Initialize ENET module */
    ENET_Init(ENET, &global_enet_handle, &config, &buf_config, mac_addr,
              sysClock);

    /* Adding Multicast Group
     * Even ksdk 2.0 API is weird.
     * ENET_AddMulticastGroup(ENET_Type *base, uint8_t *address),
     * this routine will need 64 addresses to set a multicast group.
     * So let's hack it. For more details->Kevin*/
    ENET->GAUR = 0xFFFFFFFFu;
    ENET->GALR = 0xFFFFFFFFu;

    /* Setup callback for Interrupt routines */
    ENET_SetCallback(&global_enet_handle, ethernet_event_callback, NULL);

    /* Lastly, activate the module */
    ENET_ActiveRead(ENET);
#endif // 0
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
#if 0 // TODO
    while (!(global_enet_handle.rxBdCurrent->control & ENET_BUFFDESCRIPTOR_RX_EMPTY_MASK)) {
        k64f_eth_receive(global_enet_handle.rxBdCurrent);
        update_read_buffer();
    }
#endif // 0
}


/** \brief  Interrupt Service Routine for TX IRQ
 *      If Mbed RTOS is being used, this routine will be called from the
 *      Eth_IRQ_Thread (thread context) after receiving the signal from
 *      interrupt context (SIG_TX).
 *      Otherwise, it will be called directly from the interrupt context.
 */
static void enet_tx_task(void)
{
    tx_queue_reclaim();
}


/** \brief  Callback function to receive Ethernet TX/RX Events
 *
 *  Called from the interrupt context. If we are not using mbed RTOS, we call up
 *  the ISRs, otherwise we send a corresponding signal to the responsible Thread.
 *
 *  \param[in] base  Pointer to base structure of Eth module
 *  \param[in] handle Pointer to global ethernet handle
 *  \param[in] event  Type of interrupt or any other event (what we have
 *             registered in init)
 *  \param[in] param A pointer to user provided data|parameter. [optional]
 */
#if 0 // TODO
static void ethernet_event_callback(ENET_Type *base, enet_handle_t *handle, enet_event_t event, void *param)
{
    switch (event) {
        case kENET_RxEvent:
#ifdef MBED_CONF_RTOS_PRESENT
            osSignalSet(eth_irq_thread_id, SIG_RX);
#else
            enet_rx_task();
#endif /*MBED_CONF_RTOS_PRESENT*/
            break;
        case kENET_TxEvent:
#ifdef MBED_CONF_RTOS_PRESENT
            osSignalSet(eth_irq_thread_id, SIG_TX);
#else
            enet_tx_task();
#endif /*MBED_CONF_RTOS_PRESENT*/
            break;
        default:
            break;
    }
}
#endif // 0


#ifdef MBED_CONF_RTOS_PRESENT
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
        osEvent event =  osSignalWait(0, osWaitForever);
        if (event.status != osEventSignal) {
            continue;
        }

        eth_if_lock();

        if (event.value.signals & SIG_RX) {
            enet_rx_task();
        }

        if (event.value.signals & SIG_TX) {
            enet_tx_task();
        }

        eth_if_unlock();
    }
}


/** \brief  Function creating the IRQ thread
 *
 *      Used only in case of  mbed RTOS. Creates a thread for IRQ task.
 */
static void Eth_IRQ_Thread_Create(void)
{
    static osThreadDef(Eth_IRQ_Thread, osPriorityRealtime, 512);
    eth_irq_thread_id = osThreadCreate(osThread(Eth_IRQ_Thread), NULL);
}
#endif /*MBED_CONF_RTOS_PRESENT*/
