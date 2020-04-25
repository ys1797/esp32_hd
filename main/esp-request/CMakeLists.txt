# Edit following two lines to set component requirements (see docs)
#list(APPEND COMPONENT_SRCS "esp_request.c" "req_list.c" "uri_parser.c")
#set(COMPONENT_ADD_INCLUDEDIRS "include")

idf_component_register(SRCS "esp_request.c" "req_list.c" "uri_parser.c"
                       INCLUDE_DIRS . "include"
		       REQUIRES openssl spi_flash spiffs fatfs)