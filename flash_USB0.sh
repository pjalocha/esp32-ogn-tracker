# you may need to change the port name
# you may need to change python into python3
# you may need to install pyserial python or python3 module: pip install pyserial
# you may need to install python or python3 pip: sudo apt-get install python-pip
python esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset \
write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 \
build/bootloader/bootloader.bin 0x10000 build/esp32-ogn-tracker.bin 0x8000 build/partitions.bin
