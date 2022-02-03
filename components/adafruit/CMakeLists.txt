# Edit following two lines to set component requirements (see docs)
#list(APPEND COMPONENT_SRCS "Adafruit_GFX.cpp" "Adafruit_ILI9341.cpp" "glcdfont.c" )
#set(COMPONENT_ADD_INCLUDEDIRS "include" ".")

idf_component_register(SRCS "Adafruit_GFX.cpp" "Adafruit_ILI9341.cpp" "glcdfont.c"
                       INCLUDE_DIRS . )

