# ECEN-5713 Advanced Embedded Software Development
# Author - Khyati Satta
# Date - 15 April 2023
# File Description - Makefile for the cam_capture.c 

CC = gcc
CFLAGS = -g -Wall -Werror

all: camera_driver

camera_driver: camera_driver.c
	$(CC) camera_driver.c $(CFLAGS)  -o camera_driver

clean:
	rm -rf *.o camera_driver 



