include ../Makefile.tests_common

BOARD = pulga
PORT = /dev/ttyUSB0

DRIVER ?= bmx160_i2c

USEMODULE += fmt
USEMODULE += xtimer
USEMODULE += $(DRIVER)

include $(RIOTBASE)/Makefile.include
