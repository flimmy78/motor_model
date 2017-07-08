
all:
		gcc -shared -o libregcur.so  -fPIC libregcur.c
		gcc -shared -o libregspeed.so  -fPIC libregspeed.c

clean:
		rm *.so *.o


