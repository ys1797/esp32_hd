#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp32_hd

include $(IDF_PATH)/make/project.mk

# Create file system images.
images:
	echo "Create spiffs.img ..."
	./mkspiffs/mkspiffs -c html -b 4096 -p 256 -s 0xF0000 build/spiffs.img

flashdata: images 
	echo "Flashing SPIFFS to ESP32" 
	$(ESPTOOLPY_WRITE_FLASH)  0x110000 build/spiffs.img 

flashall: flash flashdata

what:
	echo "flash             - Flash the ESP32 application." 
	echo "flashall          - Flash the ESP32 application and file systems data." 
	echo "flashdata         - Flash the file systems data." 
