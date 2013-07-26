pynrf24
=======

Python port of the RF24 library for NRF24L01+ radios



Introduction
------------


All methods were ported and most methods prototypes were kept similar. This should facilitate the adaptation of existing code.
Limitations were also ported and unfortunately some bugs may have been introduced.

For more information regarding how to use this library, check the RF24 documentation: http://maniacbug.github.io/RF24/
Most of the information there will also be valid to this library.

Tested in a BeagleBoneBlack using spi0 and NRF24L01+ radios.
Should work with not problems in a Raspberry Pi or with NRF24L01 (non +) radios.

Requirements
------------

 * Python 2
 * SPI Enabled on the device: http://hipstercircuits.com/enable-spi-with-device-tree-on-beaglebone-black-copy-paste/
 * SPI communication requires spidev:  https://pypi.python.org/pypi/spidev
 * GPIO access requires Adafruit BBIO library: https://github.com/adafruit/adafruit-beaglebone-io-python

Wiring
------

    nRF24L01+             BeagleBoneBlack
	+-+-+                    (header)
	|8|7|	1: GND      ->   P9 GND
	+-+-+	2: 3.3V     ->   P9 3.3v
	|6|5|	3: CE       ->   P9_15 (configurable)
	+-+-+	4: CSN      ->   SPI0.CS
	|4|3|	5: SCKL     ->   SPI0.SCK
	+-+-+	6: MOSI     ->   SPI0.D0
	|2|1|	7: MISO     ->   SPI0.D1
	+-+-+	8: IRQ      ->   P9_16 (configurable)

Examples
--------

Initialization:

		pipes = [ [0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2] ]

		radio = NRF24()
		radio.begin(1,0,"P9_15", "P9_16") #Set CE and IRQ pins
		radio.setRetries(15,15)
		radio.setPayloadSize(8)
		radio.setChannel(0x60)
		radio.setDataRate(NRF24.BR_250KBPS)
		radio.setPALevel(NRF24.PA_MAX)
		radio.openWritingPipe(pipes[0])
		radio.openReadingPipe(1,pipes[1])

		radio.printDetails()


Sending Data:

    buffer = ['H','E','L','L','O']
    status = radio.write(buffer)


Receiving Data:

	#Wait for data
	pipe =[0]
	while not radio.available(pipe):
		time.sleep(10000/1000000.0)

	#Receive Data
	recv_buffer = []
    radio.read(recv_buffer)

	#Print the buffer
	print recv_buffer

