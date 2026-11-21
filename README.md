# esp32s3 gio 46 if pulled high will set flash votage to 1.8v and device won't boot
# burn the efuse to disable it

espefuse --chip esp32s3 --port /dev/tty.usbmodem1101 set-flash-voltage 3.3V
