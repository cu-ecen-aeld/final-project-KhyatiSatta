# ECEN-5713 Advanced Embedded Software Development
# Author - Khyati Satta
# Date - 15 April 2023
# File Description - Makefile for the cam_capture.c 

CROSS_COMPILE =
ifeq ($(CC),)
	CC = $(CROSS_COMPILE)gcc
endif
ifeq ($(CFLAGS),)
	CFLAGS = -g -Wall -Werror
endif
ifeq ($(LDFLAGS),)
	LDFLAGS = 
endif

SRC := camera_driver.c
TARGET ?= camera_driver

all: $(TARGET)

$(TARGET) : $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) camera_driver.c $(LDFLAGS)

clean:
	-rm -f *.o $(TARGET) *.elf *.map


