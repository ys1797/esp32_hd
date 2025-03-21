# Edit following two lines to set component requirements (see docs)
#set(COMPONENT_REQUIRES spi_flash)
idf_component_register(SRCS "auth.c" "base64.c" "cgiwebsocket.c"
					"httpd.c" 
					"httpdespfs.c" 
					"httpd-freertos.c" 
					"sha1.c"
                     INCLUDE_DIRS "." "include"
		     REQUIRES spi_flash spiffs fatfs mbedtls)
