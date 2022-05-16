tar cvzf esp32-ogn-tracker-bin.tgz flash_USB0.sh flash_ACM0.sh esptool.py flash_COM?.bat esptool.py \
build/partitions.bin build/bootloader/bootloader.bin build/esp32-ogn-tracker.bin \
utils/read_log utils/serial_dump main/config.h
zip -u esp32-ogn-tracker-bin.zip flash_USB0.sh flash_ACM0.sh esptool.py flash_COM?.bat esptool.py \
build/partitions.bin build/bootloader/bootloader.bin build/esp32-ogn-tracker.bin \
utils/read_log utils/serial_dump main/config.h
