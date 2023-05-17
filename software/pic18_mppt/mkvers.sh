#!/bin/sh -e
dst=$1

rm -f $dst
echo "/* automatically generated from mkvers.sh - do not edit */" > $dst
date +'const char buildstr[] = "%d/%m/%y %H:%M";' >> $dst
