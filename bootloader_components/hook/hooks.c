#include "esp_log.h"
#include "board_cfg.h"

#include "rom/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_periph.h"

#define TAG "BTL"


// Function used to tell the linker to include this file with all its symbols.
void bootloader_hooks_include(void) { }


void bootloader_before_init(void) {
//	Keep in my mind that a lot of functions cannot be called from here
//	as system initialization has not been performed yet, including
//	BSS, SPI flash, or memory protection.

    esp_rom_gpio_pad_select_gpio(BOARD_CFG_GPIO_RLY_1);
    esp_rom_gpio_pad_select_gpio(BOARD_CFG_GPIO_RLY_2);
    GPIO_OUTPUT_SET(BOARD_CFG_GPIO_RLY_1, 0);
    GPIO_OUTPUT_SET(BOARD_CFG_GPIO_RLY_2, 0);

    esp_rom_gpio_pad_select_gpio(BOARD_CFG_GPIO_BROWNOUT);
    if (GPIO_PIN_MUX_REG[BOARD_CFG_GPIO_BROWNOUT])
        PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[BOARD_CFG_GPIO_BROWNOUT]);

    if (GPIO_INPUT_GET(BOARD_CFG_GPIO_BROWNOUT)) {
        ESP_LOGI(TAG, "Power loss detected");
        // it would be nice to return to sleep. But bootloader is not supporting it
#ifndef BOARD_DEBUG_NO_BROWNOUT
        while(1);
#endif
        return;
    }
    esp_rom_gpio_pad_select_gpio(BOARD_CFG_GPIO_LED_0);
    GPIO_OUTPUT_SET(BOARD_CFG_GPIO_LED_0, 0);
}

void bootloader_after_init(void) {

    esp_rom_gpio_pad_select_gpio(BOARD_CFG_GPIO_LED_2);
    GPIO_OUTPUT_SET(BOARD_CFG_GPIO_LED_2, 0);
}
