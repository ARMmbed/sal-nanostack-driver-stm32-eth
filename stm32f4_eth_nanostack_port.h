/*
 * Copyright (c) 2014-2016 ARM. All rights reserved.
 */

#ifndef STM32F4_ETH_NANOSTACK_PORT_H_
#define STM32F4_ETH_NANOSTACK_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

extern void arm_eth_phy_device_register(uint8_t *mac_ptr, void (*app_ipv6_init_cb)(uint8_t, int8_t));

#ifdef __cplusplus
}
#endif

#endif /* STM32F4_ETH_NANOSTACK_PORT_H_ */
