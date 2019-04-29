#!/bin/bash
#
# Script used in CI to build all configurations of mkspiffs
#

cp build/bootloader/bootloader.bin ./
cp build/partitions.bin ./
cp build/esp32_hd.bin ./
cp build/spiffs.img ./
./zip fw_esp32_v06.zip bootloader.bin partitions.bin esp32_hd.bin spiffs.img