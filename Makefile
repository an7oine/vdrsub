CFLAGS=-std=c99 -DVERBOSE -DVOBSUB

all: vdrsub

clean: 
	rm -rf vdrsub *.o

vdrsub: dvbsub.o vdrsub.o write-ps.o
	gcc dvbsub.o vdrsub.o write-ps.o -o vdrsub

dvbsub.o: dvbsub.c
	gcc $(CFLAGS) -c dvbsub.c

vdrsub.o: vdrsub.c
	gcc $(CFLAGS) -c vdrsub.c

write-ps.o: write-ps.c
	gcc $(CFLAGS) -c write-ps.c
