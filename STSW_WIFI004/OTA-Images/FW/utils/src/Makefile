# Makefile hex2bin/mot2bin

CC=gcc
CFLAGS=-I/cygdrive/c/cygwin/usr/local/ssl/include
LDFLAGS=-L/cygdrive/c/cygwin/usr/local/ssl/lib -lcrypto
CPFLAGS = -std=c99 -O2 -Wall -pedantic

# Compile
%o : %c
	gcc -c $(CPFLAGS) $< -o $@

#WIN64_GCC = x86_64-w64-mingw32-gcc
WINDOWS = i686-w64-mingw32
WIN_GCC = $(WINDOWS)-gcc
WIN_STRIP = $(WINDOWS)-strip

INSTALL_DIR = /usr/local
MAN_DIR = $(INSTALL_DIR)/man/man1

all: create_ota

create_ota: hex2bin.o common.o libcrc.o binary.o create_ota.o
	$(CC) -O2 -Wall -o create_ota hex2bin.o common.o libcrc.o binary.o create_ota.o $(CFLAGS) $(LDFLAGS)

install:
	strip create_ota	
	cp create_ota $(INSTALL_DIR)/bin

clean:
	rm -f create_ota core *.o hex2bin mot2bin 
