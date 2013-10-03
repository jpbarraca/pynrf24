Intructions
====

Copy the file BB-nRF24-SPI0-00A0.dts to the BBB.
Then compile it using the following command:

		dtc -O dtb -o BB-nRF24-SPI0-00A0.dtbo -b 0 -@ BB-nRF24-SPI0-00A0.dts

This overlay is almost ready to be used. But first it must e copied
to /lib/firmware

		cp BB-nRF24-SPI0-00A0.dtbo /lib/firmware

If everything was done correctly, the overlay can be enabled by issuing:

		echo BB-nRF24-SPI0 >  /sys/devices/bone_capemgr.8/slots

This can be made "permanent" if the line above is inserted into
/etc/rc.local

A device should appear in /dev/spidev.1.0

