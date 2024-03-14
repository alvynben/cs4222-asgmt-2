CONTIKI_PROJECT = etimer-buzzer rtimer-lightSensor rtimer-IMUSensor a2p2 a2p3
all: $(CONTIKI_PROJECT)
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
CFLAGS += -w
