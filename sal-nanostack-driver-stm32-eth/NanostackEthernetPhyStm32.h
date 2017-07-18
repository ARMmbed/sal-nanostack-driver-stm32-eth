/*
 * Copyright (c) 2016 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NANOSTACK_ETH_PHY_STM32_H_
#define NANOSTACK_ETH_PHY_STM32_H_

#include "NanostackEthernetPhy.h"

class NanostackEthernetPhyStm32 : public NanostackEthernetPhy {
public:
    NanostackEthernetPhyStm32();
    int8_t phy_register();
    void get_mac_address(uint8_t *mac);
    void set_mac_address(uint8_t *mac);
private:
    uint8_t _mac[6];
};

#endif /* NANOSTACK_ETH_PHY_STM32_H_ */
