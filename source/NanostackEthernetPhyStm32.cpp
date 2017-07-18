#include "mbed.h"
#include "NanostackEthernetPhyStm32.h"
#include "stm32_eth_nanostack_port.h"

static Semaphore enet_sem(0);

static int8_t iface_id = -1;
static bool iface_is_up = false;

static void driver_status_cb(uint8_t is_up, int8_t iface)
{
    iface_id = iface;
    iface_is_up = is_up?true:false;
    enet_sem.release();
}

NanostackEthernetPhyStm32::NanostackEthernetPhyStm32()
{
    mbed_mac_address((char *)_mac);
}

int8_t NanostackEthernetPhyStm32::phy_register()
{
    arm_eth_phy_device_register(_mac, driver_status_cb);
    enet_sem.wait();
    return iface_id;
}

void NanostackEthernetPhyStm32::get_mac_address(uint8_t *mac)
{
    memmove(mac, _mac, 6);
}

void NanostackEthernetPhyStm32::set_mac_address(uint8_t *mac)
{
    memmove(_mac, mac, 6);
}
