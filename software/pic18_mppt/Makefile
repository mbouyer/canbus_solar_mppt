ROM_BASE=600
IVECT_BASE=0x608

.SUFFIXES: .p1 .hex

.PATH: ${.CURDIR}/../../../pic18_n2k

CC= xc8-cc -mcpu=18f27q84 -mno-config -mkeep-startup -O2
CC+= -mcodeoffset=${ROM_BASE} -mreserve=rom@0x10000:0x1ffff -mreserve=ram@0x3700:0x37ff
CFLAGS= -DIVECT_BASE=${IVECT_BASE} -I${.CURDIR} -I${.CURDIR}/../../../pic18_n2k
OBJECTS= main.p1 serial.p1 i2c.p1 nmea2000.p1 ntc_tab.p1 font10x16.p1 font5x8.p1 icons16x16.p1
HEADERS= serial.h nmea2000.h nmea2000_pgn.h nmea2000_user.h i2c.h nmea2000_pic18_ecan.c ntc_tab.h font10x16.h font5x8.h icons16x16.h pac195x.h vers.h

all: solarmppt.hex

obj:
	mkdir obj

solarmppt.bin: solarmppt.hex
	hex2bin 0x300 ${.ALLSRC} ${.TARGET}
solarmppt.hex: ${OBJECTS} vers.p1
	${CC} -Wa,-a -Wl,-Map=solarmppt.map  -o ${.TARGET} ${.ALLSRC} 

${OBJECTS}: ${HEADERS} Makefile

vers.c: ${OBJECTS} ${.CURDIR}/mkvers.sh
	${.CURDIR}/mkvers.sh ${.TARGET}
	
.c.p1:
	${CC} ${CFLAGS} -c ${.IMPSRC} -o ${.TARGET}

clean:
	rm -f *.p1 *.hex
