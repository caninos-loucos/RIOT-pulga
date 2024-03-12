# name of your application
APPLICATION = default

# Use the pulga board (nrf52840dk under the hood) by default:
BOARD ?= pulga

# default serial port
PORT ?= /dev/ttyUSB0

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../../pulga-riot

FEATURES_REQUIRED += cpp
FEATURES_REQUIRED += periph_uart

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

# Modules to include:
#USEMODULE += shell
#USEMODULE += shell_cmds
USEMODULE += ps
USEMODULE += xtimer
USEMODULE += printf_float

USEPKG += minmea

include $(RIOTBASE)/Makefile.include
