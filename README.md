| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# I2S Basic PDM Mode Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example is going to show how to use the PDM TX and RX mode.

## How to Use Example

### Hardware Required

#### General

* A development board with any supported Espressif SOC chip (see `Supported Targets` table above)
* A USB cable for power supply and programming

#### PDM RX

 idf.py build && esptool.py -p /dev/tty.usbmodem2101 --no-stub load_ram build/i2s_pdm_example.bin && idf.py -p /dev/tty.usbmodem2101 monitor --no-reset
