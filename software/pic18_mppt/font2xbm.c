#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

#define __rom /**/

#include "font5x8.c"

int main(int argc, const char *argv[])
{
	int c, l;
	char fi = atoi(argv[1]);
	unsigned char out;

	const unsigned char *f=font5x8[fi];

	printf("#define f_width 5\n");
	printf("#define f_height 8\n");
	printf("static unsigned char f_bits[] = {\n");
	for (l = 0; l < 8; l++) {
		out = 0;
		for (c = 0; c < 5; c++) {
			if (f[c] & (1 << l)) {
				out |= (1 << c);
			}
		}
		printf("0x%02x, ", out);
	}
	printf("\n};\n");
	exit(0);
}
