idf_component_register(SRCS "Stellarsat-frames.c" "main.c"
                    PRIV_REQUIRES spi_flash libcorrect
                    INCLUDE_DIRS ""
                    REQUIRES driver)
