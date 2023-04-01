#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

#include "engine.xbm"

int main(int argc, const char *argv[])
{

	uint8_t out;
	int l, c;
	int byte, bit;
	int out_bit;
	for (l = 0; l < engine_height; l++) {
		for (c = 0; c < engine_width; c++) {
			int pixel = c + l * engine_width;
			byte = pixel / (sizeof(engine_bits[0]) * 8);
			bit = pixel % (sizeof(engine_bits[0]) * 8);

			if ((engine_bits[byte] >> bit) & 0x1) {
				fprintf(stderr, "O");
			} else {
				fprintf(stderr, ".");
			}
		}
		fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n");

	printf("{ ");
	for (c = 0; c < engine_width; c++) {
		out = 0;
		for (l = 0; l < engine_height; l++) {
			int pixel = c + l * engine_width;
			byte = pixel / (sizeof(engine_bits[0]) * 8);
			bit = pixel % (sizeof(engine_bits[0]) * 8);
#if 0
			fprintf(stderr, "c %d l %d byte %d bit %d of 0x%x",
			    c, l, byte, bit, engine_bits[byte]);
#endif
			out_bit = l % (sizeof(out) * 8);
			if ((engine_bits[byte] >> bit) & 0x1) {
				out |= 1 << out_bit;
				fprintf(stderr, "O");
			} else {
				fprintf(stderr, ".");
			}
#if 0
			fprintf(stderr, " out_bit %d => 0x%02x", out_bit, out);
#endif
			if (((l+1) % (sizeof(out) *8)) == 0) {
				// fprintf(stderr, " * ");
				printf("0x%02x,", out);
				out = 0;
			}
		}
		fprintf(stderr, "\n");
	}
	printf(" },\n");
	exit(0);
}
