# ECEN-5713 Advanced Embedded Software Development
# Author - Khyati Satta
# Date - 15 April 2023
# File Description - Makefile for the cam_capture.c 

CC = gcc
CFLAGS = -g -Wall -Werror

all: cam_capture

cam_capture: cam_capture.c
	$(CC) cam_capture.c $(CFLAGS)  -o cam_capture

clean:
	rm -rf *.o cam_capture 



