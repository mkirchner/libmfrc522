libmfrc522
==========

Raspberry PI C library for MFRC522, derived from the C++ version [here][mfrc522-c] and
based on the `bcm2835` library [here][bcm2835]. Includes a C example program `read.c` to read RFID cards.

[mfrc522-c]: https://github.com/villinte/RPi-RFID/
[bcm2835]: http://www.airspayce.com/mikem/bcm2835/bcm2835-1.75.tar.gz 


## Installation

### Install bcm2835

Fetch the tar-file and install it:

    tar zxvf bcm2835-1.xx.tar.gz
    cd bcm2835-1.xx
    ./configure --prefix=/opt  # or whatever your preference
    make
    sudo make check
    sudo make install


### Build libmfrc522:

	gcc -o rfid mfrc522.c read.c -std=c11 -lbcm2835 -L/opt/bcm2835/lib -I/opt/bcm2835/include


### Run

Make sure you run as root, then:

    sudo ./rfid


Pin Layout
----------

The following table shows the pin layout used:

    +-----------+----------+-------------+
    |           | MFRC522  | Raspberry Pi|
    +-----------+----------+-------------+
    | Signal    | Pin      | Pin         |
    +===========+==========+=============+
    | RST/Reset | RST      | 22          |
    +-----------+----------+-------------+
    | SPI SS    | SDA      | 24          |
    +-----------+----------+-------------+
    | SPI MOSI  | MOSI     | 19          |
    +-----------+----------+-------------+
    | SPI MISO  | MISO     | 21          |
    +-----------+----------+-------------+
    | SPI SCK   | SCK      | 23          |
    +-----------+----------+-------------+
    | 3V        | 3v       | 1           |
    +-----------+----------+-------------+
    | GND       | GND      | 25          |
    +-----------+----------+-------------+
