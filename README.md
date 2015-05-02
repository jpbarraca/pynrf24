pynrf24
=======

Pure python port of the RF24 (https://github.com/maniacbug/RF24/) library for NRF24L01+ radios through SPI. 

All API methods were ported and prototypes were kept very similar. This should facilitate the reuse of code from the STM32 or Arduino platforms.
The focus of this libary is to provide an API similar to the RF24 driver. Therefore, many things are not so pythonic, especially function names.
The actually implementation is improved from the original RF24 code base and has many bug fixes solved, as well as improved performance.

For more information regarding how to use this library, check the RF24 documentation: http://maniacbug.github.io/RF24/
Most of the information there will also be valid here.

Tested on the the BeagleBone Black, Raspberry Pi, and Banana Pi.
Should work without problems with NRF24L01 (non +) radios.

Do not forget to check this [great tutorial](http://www.diyembedded.com/tutorials/nrf24l01_0/nrf24l01_tutorial_0.pdf)
or the [original datasheet](http://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01)


Contact
-------

For any information regarding this library please create an issue.

Improvements to the code base are also welcome.

Requirements
------------

 * Python 2
 * SPI communication requires spidev:  https://pypi.python.org/pypi/spidev
 * BBB: GPIO access requires Adafruit BBIO library: https://github.com/adafruit/adafruit-beaglebone-io-python
 * BBB: If using GPIO IRQ detection, a custom version of the Adafruit library can be used https://github.com/jpbarraca/adafruit-beaglebone-io-python in order to support a timeout when waiting for the IRQ line.
 

Wiring
------

    nRF24L01+ (top view)     BeagleBoneBlack
	+-+-+                    (header)
	|8|7|	1: GND      ->   P9 GND (P9_1 and P9_2)
	+-+-+	2: 3.3V     ->   P9 3.3v (P9_3 and P9_4)
	|6|5|	3: CE       ->   P9_23 (configurable)
	+-+-+	4: CSN      ->   SPI0.CS (P9_17)
	|4|3|	5: SCKL     ->   SPI0.SCLK (P9_22)
	+-+-+	6: MOSI     ->   SPI0.D1 (P9_18)
	|2|1|	7: MISO     ->   SPI0.D0 (P9_21)
	+-+-+	8: IRQ      ->   P9_24 (configurable)

Examples
--------

Initialization:

		pipes = [ [0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2] ]

		radio = NRF24()
		radio.begin(1,0,"P9_23", "P9_24") #Set CE and IRQ pins
		radio.setRetries(15,15)
		radio.setPayloadSize(8)
		radio.setChannel(0x60)
		radio.setDataRate(NRF24.BR_250KBPS)
		radio.setPALevel(NRF24.PA_MAX)
		radio.openWritingPipe(pipes[0])
		radio.openReadingPipe(1,pipes[1])

		radio.printDetails()


Sending Data:

    status = radio.write("HELLO")


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


Caveats
-------

Performance with this driver (as well as with any other Python "driver") may be lower than expected. You will sacrifice performance for flexibility.
Python is a high level language and there is too much stuff going on (e.g, Garbage Collection) for timing constrains to be respected. There is no assurance that you can transmit at 2Mbits/s.

Benchmarks using an Nucleo 401RE and a BBB show a usable bitrate around 582kbits/s for a 2Mbit/s configuration. This was achieved using packets with 32 byte payloads, no CRC, and no Acknowledgements. According to the datasheet, for a 32 byte buffer, there will be more 65 bits of overhead, and the process will take at least 25.6us to upload the buffer, 130us for the PLL to lock, and 6us for the TX DS bit to be set. The total time spent will be of 160.5us + 25.6us + 130us + 6us = 322.1us, independently of the driver, and resulting in a usable bitrate of 795 Kb/s. Then you have to consider the latency of the SPI bus and code being executed at the uC. Actually, in this benchmark the SPI bus driver was taking 45% of the CPU.

It works great in my case. Use AS-IS, but remember that YMMV.

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

