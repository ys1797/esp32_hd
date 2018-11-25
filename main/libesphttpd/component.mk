#
# Component Makefile (for esp-idf)
#
# This Makefile should, at the very least, just include $(SDK_PATH)/make/component.mk. By default, 
# this will take the sources in this directory, compile them and link them into 
# lib(subdirectory_name).a in the build directory. This behaviour is entirely configurable,
# please read the SDK documents if you need to do this.
#

COMPONENT_SRCDIRS := core espfs util
COMPONENT_ADD_INCLUDEDIRS := core espfs util include
COMPONENT_ADD_LDFLAGS := -llibesphttpd

COMPONENT_EXTRA_CLEAN := mkespfsimage/*

CFLAGS += -DFREERTOS

