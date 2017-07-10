OPT = -Wall -O0


all:
		gcc $(OPT) -shared -o libregcur.so  -fPIC libregcur.c cos_tb.c
		gcc $(OPT) -shared -o libregspeed.so  -fPIC libregspeed.c cos_tb.c


clean:
		rm *.so *.o


