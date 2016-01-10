all:
	nasm -f win32 KalmanFilter.asm -o KalmanFilter.obj
	gcc -Wall -c KalmanFilter.c

	ar rcs KalmanFilter.lib KalmanFilter.obj KalmanFilter.o
	ar -t KalmanFilter.lib KalmanFilter.obj KalmanFilter.o
	gcc kalmanTest.c -o kalmanTest.exe   KalmanFilter.lib
clean:
	rm -rf KalmanFilter.obj KalmanFilter.lib kalmanTest.exe KalmanFilter.o