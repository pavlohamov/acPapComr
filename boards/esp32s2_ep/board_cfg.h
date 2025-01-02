/*
 * board_cfg.h
 *
 *  Created on: Jan 12, 2024
 *      Author: pavlo.hamov@sigan.tech
 */

#pragma once

#define BOARD_CFG_ID "esp32s2 shapa"
#define BOARD_CFG_REVISION 0

#define BOARD_CFG_DISPLAY

#if 0
#define BOARD_CFG_DISPLAY_WIDTH 250
#define BOARD_CFG_DISPLAY_HEIGHT 120
#else
#define BOARD_CFG_DISPLAY_WIDTH 120
#define BOARD_CFG_DISPLAY_HEIGHT 250
#endif

//#define BOARD_CFG_GPIO_BROWNOUT 33

#define BOARD_CFG_GPIO_NPGOOD 13
#define BOARD_CFG_GPIO_FORCE_PWR 12


#define BOARD_CFG_GPIO_LCD_BUSY  21
#define BOARD_CFG_GPIO_LCD_RST   26
#define BOARD_CFG_GPIO_LCD_MOSI  15
#define BOARD_CFG_GPIO_LCD_CLK   14
#define BOARD_CFG_GPIO_LCD_DC    16
#define BOARD_CFG_GPIO_LCD_CS    17

#define BOARD_CFG_GPIO_RLY_1 18
#define BOARD_CFG_GPIO_RLY_2 1

#define BOARD_CFG_I2C_BUS I2C_NUM_0
#define BOARD_CFG_GPIO_SDA ((gpio_num_t)7)
#define BOARD_CFG_GPIO_SCL ((gpio_num_t)6)
#define BOARD_CFG_FRAM_ADDR 0x50

#define BOARD_CFG_GPIO_LED_0 2
#define BOARD_CFG_GPIO_LED_1 3
#define BOARD_CFG_GPIO_LED_2 4
#define BOARD_CFG_GPIO_LED_3 5
