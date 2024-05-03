all:
	gcc -o rfid mfrc522.c read.c -O0 -g -std=c11 -lbcm2835 -L/opt/bcm2835/lib -I/opt/bcm2835/include
