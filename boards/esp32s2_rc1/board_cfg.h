/*
 * board_cfg.h
 *
 *  Created on: Jan 12, 2024
 *      Author: pavlo.hamov@sigan.tech
 */

#pragma once

#define BOARD_CFG_ID "esp32s2 rc1"
#define BOARD_CFG_REVISION 0

#define BOARD_CFG_DISPLAY

#if 0
#define BOARD_CFG_DISPLAY_WIDTH 250
#define BOARD_CFG_DISPLAY_HEIGHT 120
#else
#define BOARD_CFG_DISPLAY_WIDTH 120
#define BOARD_CFG_DISPLAY_HEIGHT 250
#endif

//#define BOARD_CFG_GPIO_BROWNOUT 26

#define BOARD_CFG_GPIO_NPGOOD 46
#define BOARD_CFG_GPIO_FORCE_PWR 14


#define BOARD_CFG_GPIO_LCD_BUSY  39
#define BOARD_CFG_GPIO_LCD_RST   38
#define BOARD_CFG_GPIO_LCD_MOSI  34
#define BOARD_CFG_GPIO_LCD_CLK   35
#define BOARD_CFG_GPIO_LCD_DC    37
#define BOARD_CFG_GPIO_LCD_CS    36

#define BOARD_CFG_GPIO_RLY_1 17
#define BOARD_CFG_GPIO_RLY_2 18

#define BOARD_CFG_I2C_BUS I2C_NUM_0
#define BOARD_CFG_GPIO_SDA ((gpio_num_t)40)
#define BOARD_CFG_GPIO_SCL ((gpio_num_t)41)
#define BOARD_CFG_FRAM_ADDR 0x50

#define BOARD_CFG_GPIO_LED_0 1
#define BOARD_CFG_GPIO_LED_1 2
#define BOARD_CFG_GPIO_LED_2 42
#define BOARD_CFG_GPIO_LED_3 45


#define BOARD_CFG_GPIO_USER_BTN 21
