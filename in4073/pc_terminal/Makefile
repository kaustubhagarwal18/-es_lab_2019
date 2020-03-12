
CC= gcc
CFLAGS = -g -Wall
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) RS232_serial.c term_serial.c commands.c pc_terminal.c -lm -o $(EXEC)

run: all
	$(EXEC)