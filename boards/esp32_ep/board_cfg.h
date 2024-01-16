/*
 * board_cfg.h
 *
 *  Created on: Jan 12, 2024
 *      Author: pavlo.hamov@sigan.tech
 */

#pragma once

#define BOARD_CFG_ID "esp32 liligo T5 V2.3.1"
#define BOARD_CFG_REVISION 0


#define BOARD_CFG_DISPLAY

#define BOARD_CFG_DISPLAY_WIDTH 250
#define BOARD_CFG_DISPLAY_HEIGHT 128


#define BOARD_CFG_GPIO_LCD_BUSY  4
#define BOARD_CFG_GPIO_LCD_RST  16
#define BOARD_CFG_GPIO_LCD_MOSI 23
#define BOARD_CFG_GPIO_LCD_CLK  18
#define BOARD_CFG_GPIO_LCD_DC   17
#define BOARD_CFG_GPIO_LCD_CS    5


#define BOARD_CFG_GPIO_BTN 39


#define BOARD_CFG_GPIO_BROWNOUT 36

#define BOARD_CFG_GPIO_RLY_1 14
#define BOARD_CFG_GPIO_RLY_2 2

#define BOARD_CFG_I2C_BUS I2C_NUM_0
#define BOARD_CFG_GPIO_SDA 21
#define BOARD_CFG_GPIO_SCL 22
#define BOARD_CFG_FRAM_ADDR 0x50

#define BOARD_CFG_GPIO_LED_0 0
//#define BOARD_CFG_GPIO_LED_1 12
#define BOARD_CFG_GPIO_LED_2 13
//#define BOARD_CFG_GPIO_LED_3 15