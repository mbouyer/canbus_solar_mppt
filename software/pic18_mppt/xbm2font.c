#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

#include "warn.xbm"

int main(int argc, const char *argv[])
{

	uint8_t out;
	int l, c;
	int byte, bit;
	int out_bit;

	for (c = 0; c < warn_width; c++) {
		out = 0;
		for (l = 0; l < warn_width; l++) {
			byte = (c + l * warn_width) / (sizeof(warn_bits[0]) * 8);
			bit = c % (sizeof(warn_bits[0] * 8));
			fprintf(stderr, "c %d l %d byte %d bit %d of 0x%x",
			    c, l, byte, bit, warn_bits[byte]);
			out_bit = l % (sizeof(out) * 8);
			if ((warn_bits[byte] >> bit) & 0x1)
				out |= 1 << out_bit;
			fprintf(stderr, " out_bit %d => 0x%02x", out_bit, out);
			if (((l+1) % (sizeof(out) *8)) == 0) {
				fprintf(stderr, " * ");
				printf("0x%02x,", out);
				out = 0;
			}
			fprintf(stderr, "\n");
		}
	}
	printf("\n");
	exit(0);
}
