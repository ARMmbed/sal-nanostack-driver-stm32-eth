# Prototype Ethernet Driver for STM32 Nucleo Boards #

Currently supported boards:
 * [NUCLEO-F429ZI](http://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-f429zi.html)

**Note**, in order to use Ethernet on the `NUCLEO_F429ZI` development board together with `SPI1` over the board's Arduino connectors (e.g. for the [X-NUCLEO-IDS01A4](https://github.com/ARMmbed/stm-spirit1-rf-driver) expansion board) you need to perform the following HW modifications on the development board:
 * **Open** solder bridge `SB121`
 * **Close** solder bridge `SB122`
 
This driver is used with 6LoWPAN stack.
