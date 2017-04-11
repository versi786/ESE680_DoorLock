

all: crc32.c crc.c
	gcc -Wall -g  -c crc.c -o crc.o
	gcc -o crc crc.o
	make remove

remove:
	rm -f *.o

clean: remove
	rm -f crc
