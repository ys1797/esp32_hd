#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp32_hd

include $(IDF_PATH)/make/project.mk

# Create file system images.
images:
	echo "Create spiffs.img ..."
	./mkspiffs/mkspiffs -c filesystem -b 4096 -p 256 -s 0x80000 build/spiffs.img
