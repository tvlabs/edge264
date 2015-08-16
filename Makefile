CC ?= clang

edge264_test: edge264_test.c edge264.c edge264_cabac.o
	$(CC) -march=native -std=gnu99 -O3 -DTRACE=2 -o edge264_test edge264_test.c edge264_cabac.o

edge264_cabac.o: edge264_cabac.c edge264_common.h edge264.h
	$(CC) -march=native -std=gnu99 -O3 -DTRACE=2 -c edge264_cabac.c
