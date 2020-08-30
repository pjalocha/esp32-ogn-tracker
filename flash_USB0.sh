python esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset \
write_flash -u --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 \
build/bootloader/bootloader.bin 0x10000 build/app-template.bin 0x8000 build/partitions.bin
