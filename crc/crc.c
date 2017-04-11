#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// CRC FROM: http://stackoverflow.com/questions/28162496/c-library-checksum-for-binary-file-in-windows-and-linux
#include "crc32.c"

typedef struct fh {
  uint32_t crc;
  uint32_t size;
  uint32_t sw_version;
  uint32_t hw_version;
} firmware_header;

int main (int argc,char *argv[]){
        uint32_t crc;
        uint32_t size;
        uint32_t sw_version;
        uint32_t hw_version;
        int fd;
        off_t len;
        int (*cfncn)(int, uint32_t *, off_t *);

        if (argc != 5) {
          printf("Usage: crc <binary file> <sw version> <hw version> <output file>\n");
        }

        // crc32
        cfncn = crc32;
        fd = STDIN_FILENO;
        if ((fd = open(argv[1], O_RDONLY, 0)) < 0) {
          printf("Couldn't open file");
        }
        if (cfncn(fd, &crc, &len)) {
          printf("Couldn't calculate crc32");
        }
        printf("crc: %u\n", crc);

        // size of file
        FILE *infile = fopen(argv[1], "r");
        fseek (infile, 0, SEEK_END);
        size = ftell(infile);
        fclose(infile);
        printf("size: %d\n", size);

        //versions
        sw_version = atoi(argv[2]);
        hw_version = atoi(argv[3]);
        printf ("sw_version: %d\n", sw_version);
        printf ("hw_version: %d\n", hw_version);

        firmware_header header;
        header.crc = crc;
        header.size = size;
        header.sw_version = sw_version;
        header.hw_version = hw_version;

        FILE *outfile = fopen(argv[4], "wb");
        fwrite(&header, sizeof(header), 1, outfile);
        fclose(outfile);
        printf("done");

        return 0;

}
