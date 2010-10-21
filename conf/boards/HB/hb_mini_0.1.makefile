#
# tiny_1.1.makefile
#
# http://paparazzi.enac.fr/wiki/Tiny_v1.1
#


include $(PAPARAZZI_SRC)/conf/boards/tiny_2.11.makefile


BOARD=HB/hb_mini
BOARD_VERSION=0.1

#BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

# TODO: update syntax
BOARD_CFG = \"HB/hb_mini_0.1.h\"

GPS_UART_NR	= 1
MODEM_UART_NR = 0
