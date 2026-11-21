#include "esp_log.h"
#include "board_cfg.h"

#include "esp_rom_gpio.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/soc.h"

#define TAG "BTL"


// Function used to tell the linker to include this file with all its symbols.
void bootloader_hooks_include(void) { }


// No HAL/driver here: ROM pad select + raw GPIO registers (soc is always in the bootloader).
// GPIO_OUTPUT_SET is gone from rom/gpio.h on S3, and ROM can't drive pins >= 32 there.
static inline void pin_low(uint32_t gpio) {
	esp_rom_gpio_pad_select_gpio(gpio);                                  // IO_MUX -> GPIO function
	esp_rom_gpio_connect_out_signal(gpio, SIG_GPIO_OUT_IDX, false, false); // driven by GPIO_OUT reg
	if (gpio < 32) {
		REG_WRITE(GPIO_OUT_W1TC_REG, BIT(gpio));          // level 0 first...
		REG_WRITE(GPIO_ENABLE_W1TS_REG, BIT(gpio));       // ...then enable the driver
	} else {
		REG_WRITE(GPIO_OUT1_W1TC_REG, BIT(gpio - 32));
		REG_WRITE(GPIO_ENABLE1_W1TS_REG, BIT(gpio - 32));
	}
}

static void relays_off(void) {
#ifdef BOARD_CFG_GPIO_RLY_1
	pin_low(BOARD_CFG_GPIO_RLY_1);
#endif
#ifdef BOARD_CFG_GPIO_RLY_2
	pin_low(BOARD_CFG_GPIO_RLY_2);
#endif
#ifdef BOARD_CFG_GPIO_RLY_1_H
	pin_low(BOARD_CFG_GPIO_RLY_1_H);
#endif
#ifdef BOARD_CFG_GPIO_RLY_1_L
	pin_low(BOARD_CFG_GPIO_RLY_1_L);
#endif
#ifdef BOARD_CFG_GPIO_RLY_2_H
	pin_low(BOARD_CFG_GPIO_RLY_2_H);
#endif
#ifdef BOARD_CFG_GPIO_RLY_2_L
	pin_low(BOARD_CFG_GPIO_RLY_2_L);
#endif
}

void bootloader_before_init(void) {
	relays_off();
}

void bootloader_after_init(void) {
}
