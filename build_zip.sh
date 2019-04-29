#!/bin/bash
#
# Script used in CI to build all configurations of mkspiffs
#

cp build/bootloader/bootloader.bin ./bin/
cp build/partitions.bin ./bin/
cp build/esp32_hd.bin ./bin/
cp build/spiffs.img ./bin/
./zip -j ./bin/fw_esp32_v06.zip ./bin/bootloader.bin ./bin/partitions.bin ./bin/esp32_hd.bin ./bin/spiffs.img