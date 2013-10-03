pynrf24
=======

Python port of the RF24 (https://github.com/maniacbug/RF24/) library for NRF24L01+ radios


Introduction
------------

This is a port of the library developed by maniacbug for the NRF24L01 radios with Arduino.

All methods were ported and most methods prototypes were kept similar. This should facilitate the adaptation of existing code.
Limitations were also ported and unfortunately some bugs may have been introduced.

For more information regarding how to use this library, check the RF24 documentation: http://maniacbug.github.io/RF24/
Most of the information there will also be valid to this library.

Tested in a BeagleBoneBlack using spi0 and NRF24L01+ radios.
Should work without problems in a Raspberry Pi or with NRF24L01 (non +) radios.

Do not forget to check this [great tutorial](http://www.diyembedded.com/tutorials/nrf24l01_0/nrf24l01_tutorial_0.pdf)
or the [original datasheet](http://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01)


Contact
-------

For any information regarding this library you can contact me at jpbarraca at gmail


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
	+-+-+	6: MOSI     ->   SPI0.D1
	|2|1|	7: MISO     ->   SPI0.D0
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


License
-------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

