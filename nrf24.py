#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# Python port of Maniacbug NRF24L01 library
# Author: Joao Paulo Barraca <jpbarraca@gmail.com>
#
# BeagleBoneBlack and Raspberry Pi use different GPIO access methods.
# Select the most appropriate for you by uncommenting one of the 
# two imports.
# For Raspberry Pi
#import Ras.GPIO as GPIO

#For BBBB
import Adafruit_BBIO.GPIO as GPIO

import spidev
import time
import sys


def _BV(x):
    return 1 << x


class NRF24:
    MAX_CHANNEL = 127
    MAX_PAYLOAD_SIZE = 32

    # PA Levels
    PA_MIN = 0
    PA_LOW = 1
    PA_HIGH = 2
    PA_MAX = 3
    PA_ERROR = 4

    # Bit rates
    BR_1MBPS = 0
    BR_2MBPS = 1
    BR_250KBPS = 2

    # CRC
    CRC_DISABLED = 0
    CRC_8 = 1
    CRC_16 = 2
    CRC_ENABLED = 3

    # Registers
    CONFIG = 0x00
    EN_AA = 0x01
    EN_RXADDR = 0x02
    SETUP_AW = 0x03
    SETUP_RETR = 0x04
    RF_CH = 0x05
    RF_SETUP = 0x06
    STATUS = 0x07
    OBSERVE_TX = 0x08
    CD = 0x09
    RX_ADDR_P0 = 0x0A
    RX_ADDR_P1 = 0x0B
    RX_ADDR_P2 = 0x0C
    RX_ADDR_P3 = 0x0D
    RX_ADDR_P4 = 0x0E
    RX_ADDR_P5 = 0x0F
    TX_ADDR = 0x10
    RX_PW_P0 = 0x11
    RX_PW_P1 = 0x12
    RX_PW_P2 = 0x13
    RX_PW_P3 = 0x14
    RX_PW_P4 = 0x15
    RX_PW_P5 = 0x16
    FIFO_STATUS = 0x17
    DYNPD = 0x1C
    FEATURE = 0x1D


    # Bit Mnemonics */
    MASK_RX_DR = 6
    MASK_TX_DS = 5
    MASK_MAX_RT = 4
    EN_CRC = 3
    CRCO = 2
    PWR_UP = 1
    PRIM_RX = 0
    ENAA_P5 = 5
    ENAA_P4 = 4
    ENAA_P3 = 3
    ENAA_P2 = 2
    ENAA_P1 = 1
    ENAA_P0 = 0
    ERX_P5 = 5
    ERX_P4 = 4
    ERX_P3 = 3
    ERX_P2 = 2
    ERX_P1 = 1
    ERX_P0 = 0
    AW = 0
    ARD = 4
    ARC = 0
    PLL_LOCK = 4
    RF_DR = 3
    RF_PWR = 6
    RX_DR = 6
    TX_DS = 5
    MAX_RT = 4
    RX_P_NO = 1
    TX_FULL = 0
    PLOS_CNT = 4
    ARC_CNT = 0
    TX_REUSE = 6
    FIFO_FULL = 5
    TX_EMPTY = 4
    RX_FULL = 1
    RX_EMPTY = 0
    DPL_P5 = 5
    DPL_P4 = 4
    DPL_P3 = 3
    DPL_P2 = 2
    DPL_P1 = 1
    DPL_P0 = 0
    EN_DPL = 2
    EN_ACK_PAY = 1
    EN_DYN_ACK = 0

    # Instruction Mnemonics
    R_REGISTER = 0x00
    W_REGISTER = 0x20
    REGISTER_MASK = 0x1F
    ACTIVATE = 0x50
    R_RX_PL_WID = 0x60
    R_RX_PAYLOAD = 0x61
    W_TX_PAYLOAD = 0xA0
    W_ACK_PAYLOAD = 0xA8
    FLUSH_TX = 0xE1
    FLUSH_RX = 0xE2
    REUSE_TX_PL = 0xE3
    NOP = 0xFF


    # Non-P omissions
    LNA_HCURR = 0x00

    # P model memory Map
    RPD = 0x09

    # P model bit Mnemonics
    RF_DR_LOW = 5
    RF_DR_HIGH = 3
    RF_PWR_LOW = 1
    RF_PWR_HIGH = 2

    # Signal Mnemonics
    LOW = 0
    HIGH = 1

    datarate_e_str_P = ["1MBPS", "2MBPS", "250KBPS"]
    model_e_str_P = ["nRF24L01", "nRF24l01+"]
    crclength_e_str_P = ["Disabled", "8 bits", "16 bits"]
    pa_dbm_e_str_P = ["PA_MIN", "PA_LOW", "PA_MED", "PA_HIGH"]
    child_pipe = [RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5]

    child_payload_size = [RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5]
    child_pipe_enable = [ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5]

    def __init__(self):
        self.ce_pin = "P9_15"
        self.irq_pin = "P9_16"
        self.channel = 76
        self.data_rate = NRF24.BR_1MBPS
        self.wide_band = False # 2Mbs data rate in use?
        self.p_variant = False # False for RF24L01 and true for RF24L01P
        self.payload_size = 5 #*< Fixed size of payloads
        self.ack_payload_available = False #*< Whether there is an ack payload waiting
        self.dynamic_payloads_enabled = False #*< Whether dynamic payloads are enabled.
        self.ack_payload_length = 5 #*< Dynamic size of pending ack payload.
        self.pipe0_reading_address = None #*< Last address set on pipe 0 for reading.
        self.spidev = None

    def ce(self, level):
        if level == NRF24.HIGH:
            GPIO.output(self.ce_pin, GPIO.HIGH)
        else:
            GPIO.output(self.ce_pin, GPIO.LOW)
        return

    def irqWait(self):
        # A race condition may occur here.
        # TODO: Should set a timeout
        if GPIO.input(self.irq_pin) == 0:
            return

        GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING)

    def read_register(self, reg, blen=1):
        buf = [NRF24.R_REGISTER | ( NRF24.REGISTER_MASK & reg )]
        for col in range(blen):
            buf.append(NRF24.NOP)

        resp = self.spidev.xfer2(buf)
        if blen == 1:
            return resp[1]

        return resp[1:blen + 1]

    def write_register(self, reg, value, length=-1):
        buf = [NRF24.W_REGISTER | ( NRF24.REGISTER_MASK & reg )]
        if isinstance(value, (int, long)):
            if length < 0:
                length = 1

            length = min(4, length)
            for i in range(length):
                buf.insert(1, int(value & 0xff))
                value >>= 8

        elif isinstance(value, list):
            if length < 0:
                length = len(value)

            for i in range(min(len(value), length)):
                buf.append(int(value[len(value) - i - 1] & 0xff))
        else:
            raise Exception("Value must be int or list")

        return self.spidev.xfer2(buf)[0]


    def write_payload(self, buf):
        data_len = min(self.payload_size, len(buf))
        blank_len = 0
        if not self.dynamic_payloads_enabled:
            blank_len = self.payload_size - data_len

        txbuffer = [NRF24.W_TX_PAYLOAD]
        for n in buf:
            t = type(n)
            if t is str:
                txbuffer.append(ord(n))
            elif t is int:
                txbuffer.append(n)
            else:
                raise Exception("Only ints and chars are supported: Found " + str(t))

        if blank_len != 0:
            blank = [0x00 for i in range(blank_len)]
            buf.extend(blank)

        return self.spidev.xfer2(txbuffer)


    def read_payload(self, buf):
        data_len = min(self.payload_size, len(buf))
        blank_len = 0
        if not self.dynamic_payloads_enabled:
            blank_len = self.payload_size - data_len

        txbuffer = [NRF24.NOP for i in range(0, blank_len + data_len + 1)]
        txbuffer[0] = NRF24.R_RX_PAYLOAD

        payload = self.spidev.xfer2(txbuffer)
        del buf[:]
        buf.extend(payload[1:])
        return 0

    def flush_rx(self):
        return self.spidev.xfer2([NRF24.FLUSH_RX])[0]

    def flush_tx(self):
        return self.spidev.xfer2([NRF24.FLUSH_TX])[0]

    def get_status(self):
        return self.spidev.xfer2([NRF24.NOP])[0]

    def print_status(self, status):
        status_str = "STATUS\t = 0x{0:02x} RX_DR={1:x} TX_DS={2:x} MAX_RT={3:x} RX_P_NO={4:x} TX_FULL={5:x}\r\n".format(
            status,
            1 if status & _BV(NRF24.RX_DR) else 0,
            1 if status & _BV(NRF24.TX_DS) else 0,
            1 if status & _BV(NRF24.MAX_RT) else 0,
            ((status >> NRF24.RX_P_NO) & int("111", 2)),
            1 if status & _BV(NRF24.TX_FULL) else 0)

        print status_str

    def print_observe_tx(self, value):
        tx_str = "OBSERVE_TX=0x{0:02x}: POLS_CNT={2:x} ARC_CNT={2:x}\r\n".format(
            value,
            (value >> NRF24.PLOS_CNT) & int("1111",2),
            (value >> NRF24.ARC_CNT)  & int("1111",2)
            )
        print tx_str

    def print_byte_register(self, name, reg, qty=1):
        extra_tab = '\t' if len(name) < 8 else 0
        print "%s\t%c =" % (name, extra_tab),
        while qty > 0:
            print "0x%02x" % (self.read_register(reg)),
            qty -= 1
            reg += 1

        print ""

    def print_address_register(self, name, reg, qty=1):
        extra_tab = '\t' if len(name) < 8 else 0
        print "%s\t%c =" % (name, extra_tab),

        while qty > 0:
            qty -= 1
            buf = reversed(self.read_register(reg, 5))
            reg += 1
            sys.stdout.write(" 0x"),
            for i in buf:
                sys.stdout.write("%02x" % i)

        print ""


    def setChannel(self, channel):
        self.channel = min(max(0, channel), NRF24.MAX_CHANNEL)
        self.write_register(NRF24.RF_CH, self.channel)

    def getChannel(self):
        return self.read_register(NRF24.RF_CH)

    def setPayloadSize(self, size):
        self.payload_size = min(max(size, 1), NRF24.MAX_PAYLOAD_SIZE)

    def getPayloadSize(self):
        return self.payload_size

    def printDetails(self):
        self.print_status(self.get_status())
        self.print_address_register("RX_ADDR_P0-1", NRF24.RX_ADDR_P0, 2)
        self.print_byte_register("RX_ADDR_P2-5", NRF24.RX_ADDR_P2, 4)
        self.print_address_register("TX_ADDR", NRF24.TX_ADDR)

        self.print_byte_register("RX_PW_P0-6", NRF24.RX_PW_P0, 6)
        self.print_byte_register("EN_AA", NRF24.EN_AA)
        self.print_byte_register("EN_RXADDR", NRF24.EN_RXADDR)
        self.print_byte_register("RF_CH", NRF24.RF_CH)
        self.print_byte_register("RF_SETUP", NRF24.RF_SETUP)
        self.print_byte_register("CONFIG", NRF24.CONFIG)
        self.print_byte_register("DYNPD/FEATURE", NRF24.DYNPD, 2)

        #
        print "Data Rate\t = %s" % NRF24.datarate_e_str_P[self.getDataRate()]
        print "Model\t\t = %s" % NRF24.model_e_str_P[self.isPVariant()]
        print "CRC Length\t = %s" % NRF24.crclength_e_str_P[self.getCRCLength()]
        print "PA Power\t = %s" % NRF24.pa_dbm_e_str_P[self.getPALevel()]

    def begin(self, major, minor, ce_pin, irq_pin):
        # Initialize SPI bus
        self.spidev = spidev.SpiDev()
        self.spidev.open(major, minor)
        self.ce_pin = ce_pin
        self.irq_pin = irq_pin

        GPIO.setup(self.ce_pin, GPIO.OUT)
        GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        time.sleep(5 / 1000000.0)

        # Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
        # WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
        # sizes must never be used. See documentation for a more complete explanation.
        self.write_register(NRF24.SETUP_RETR, (int('0100', 2) << NRF24.ARD) | (int('1111', 2) << NRF24.ARC))

        # Restore our default PA level
        self.setPALevel(NRF24.PA_MAX)

        # Determine if this is a p or non-p RF24 module and then
        # reset our data rate back to default value. This works
        # because a non-P variant won't allow the data rate to
        # be set to 250Kbps.
        if self.setDataRate(NRF24.BR_250KBPS):
            self.p_variant = True

        # Then set the data rate to the slowest (and most reliable) speed supported by all
        # hardware.
        self.setDataRate(NRF24.BR_1MBPS)

        # Initialize CRC and request 2-byte (16bit) CRC
        self.setCRCLength(NRF24.CRC_16)

        # Disable dynamic payloads, to match dynamic_payloads_enabled setting
        self.write_register(NRF24.DYNPD, 0)

        # Reset current status
        # Notice reset and flush is the last thing we do
        self.write_register(NRF24.STATUS, _BV(NRF24.RX_DR) | _BV(NRF24.TX_DS) | _BV(NRF24.MAX_RT))

        # Set up default configuration.  Callers can always change it later.
        # This channel should be universally safe and not bleed over into adjacent
        # spectrum.
        self.setChannel(self.channel)

        # Flush buffers
        self.flush_rx()
        self.flush_tx()

    def startListening(self):
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) | _BV(NRF24.PWR_UP) | _BV(NRF24.PRIM_RX))
        self.write_register(NRF24.STATUS, _BV(NRF24.RX_DR) | _BV(NRF24.TX_DS) | _BV(NRF24.MAX_RT))

        # Restore the pipe0 address, if exists
        if self.pipe0_reading_address:
            self.write_register(self.RX_ADDR_P0, self.pipe0_reading_address, 5)

        # Go!
        self.ce(NRF24.HIGH)

        # wait for the radio to come up (130us actually only needed)
        time.sleep(130 / 1000000.0)

    def stopListening(self):
        self.ce(NRF24.LOW)
        self.flush_tx()
        self.flush_rx()

    def powerDown(self):
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) & ~_BV(NRF24.PWR_UP))

    def powerUp(self):
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) & _BV(NRF24.PWR_UP))
        time.sleep(150 / 1000000.0)

    def write(self, buf):
        # Begin the write
        self.startWrite(buf)

        timeout = self.getMaxTimeout() #s to wait for timeout
        sent_at = time.time()

        while True:
            status = self.read_register(NRF24.OBSERVE_TX, 1)
            if (status & (_BV(NRF24.TX_DS) | _BV(NRF24.MAX_RT))) or (time.time() - sent_at > timeout ):
                break
            time.sleep(10 / 1000000.0) 

        what = self.whatHappened()

        result = what['tx_ok']

        # Handle the ack packet
        if what['rx_ready']:
            self.ack_payload_length = self.getDynamicPayloadSize()

        return result

    def startWrite(self, buf):
        # Transmitter power-up
        self.write_register(NRF24.CONFIG, (self.read_register(NRF24.CONFIG) | _BV(NRF24.PWR_UP) ) & ~_BV(NRF24.PRIM_RX))

        # Send the payload
        self.write_payload(buf)

        # Allons!
        self.ce(NRF24.HIGH)
        time.sleep(10 / 1000000.0)
        self.ce(NRF24.LOW)

    def getDynamicPayloadSize(self):
        return self.spidev.xfer2([NRF24.R_RX_PL_WID, NRF24.NOP])[1]

    def available(self, pipe_num=None, irq_wait=False):
        if not pipe_num:
            pipe_num = []

        status = self.get_status()
        result = False

        if irq_wait:
            self.irqWait()

        # Sometimes the radio specifies that there is data in one pipe but
        # doesn't set the RX flag...
        if status & _BV(NRF24.RX_DR) or (status & 0b00001110 != 0b00001110):
            result = True

        if result:
            # If the caller wants the pipe number, include that
            if len(pipe_num) >= 1:
                pipe_num[0] = ( status >> NRF24.RX_P_NO ) & 0b00000111

                # Clear the status bit

                # ??? Should this REALLY be cleared now?  Or wait until we
                # actually READ the payload?
        self.write_register(NRF24.STATUS, _BV(NRF24.RX_DR))

        # Handle ack payload receipt
        if status & _BV(NRF24.TX_DS):
            self.write_register(NRF24.STATUS, _BV(NRF24.TX_DS))

        return result

    def read(self, buf):
        # Fetch the payload
        self.read_payload(buf)

        # was this the last of the data available?
        return self.read_register(NRF24.FIFO_STATUS) & _BV(NRF24.RX_EMPTY)

    def whatHappened(self):
        # Read the status & reset the status in one easy call
        # Or is that such a good idea?
        status = self.write_register(NRF24.STATUS, _BV(NRF24.RX_DR) | _BV(NRF24.TX_DS) | _BV(NRF24.MAX_RT))

        # Report to the user what happened
        tx_ok = status & _BV(NRF24.TX_DS)
        tx_fail = status & _BV(NRF24.MAX_RT)
        rx_ready = status & _BV(NRF24.RX_DR)
        return {'tx_ok': tx_ok, "tx_fail": tx_fail, "rx_ready": rx_ready}

    def openWritingPipe(self, value):
        # Note that the NRF24L01(+)
        # expects it LSB first.

        self.write_register(NRF24.RX_ADDR_P0, value, 5)
        self.write_register(NRF24.TX_ADDR, value, 5)

        max_payload_size = 32
        self.write_register(NRF24.RX_PW_P0, min(self.payload_size, max_payload_size))

    def openReadingPipe(self, child, address):
        # If this is pipe 0, cache the address.  This is needed because
        # openWritingPipe() will overwrite the pipe 0 address, so
        # startListening() will have to restore it.
        if child == 0:
            self.pipe0_reading_address = address

        if child <= 6:
            # For pipes 2-5, only write the LSB
            if child < 2:
                self.write_register(NRF24.child_pipe[child], address, 5)
            else:
                self.write_register(NRF24.child_pipe[child], address, 1)

            self.write_register(NRF24.child_payload_size[child], self.payload_size)

            # Note it would be more efficient to set all of the bits for all open
            # pipes at once.  However, I thought it would make the calling code
            # more simple to do it this way.
            self.write_register(NRF24.EN_RXADDR,
                                self.read_register(NRF24.EN_RXADDR) | _BV(NRF24.child_pipe_enable[child]))


    def closeReadingPipe(self, pipe):
        self.write_register(NRF24.EN_RXADDR,
            self.read_register(EN_RXADDR) & ~_BV(NRF24.child_pipe_enable[pipe]))


    def toggle_features(self):
        buf = [NRF24.ACTIVATE, 0x73]
        self.spidev.xfer2(buf)

    def enableDynamicPayloads(self):
        # Enable dynamic payload throughout the system
        self.write_register(NRF24.FEATURE, self.read_register(NRF24.FEATURE) | _BV(NRF24.EN_DPL))

        # If it didn't work, the features are not enabled
        if not self.read_register(NRF24.FEATURE):
            # So enable them and try again
            self.toggle_features()
            self.write_register(NRF24.FEATURE, self.read_register(NRF24.FEATURE) | _BV(NRF24.EN_DPL))

        # Enable dynamic payload on all pipes

        # Not sure the use case of only having dynamic payload on certain
        # pipes, so the library does not support it.
        self.write_register(NRF24.DYNPD, self.read_register(NRF24.DYNPD) | _BV(NRF24.DPL_P5) | _BV(NRF24.DPL_P4) | _BV(
            NRF24.DPL_P3) | _BV(NRF24.DPL_P2) | _BV(NRF24.DPL_P1) | _BV(NRF24.DPL_P0))

        self.dynamic_payloads_enabled = True


    def enableAckPayload(self):
        # enable ack payload and dynamic payload features
        self.write_register(NRF24.FEATURE,
                            self.read_register(NRF24.FEATURE) | _BV(NRF24.EN_ACK_PAY) | _BV(NRF24.EN_DPL))

        # If it didn't work, the features are not enabled
        if not self.read_register(NRF24.FEATURE):
            # So enable them and try again
            self.toggle_features()
            self.write_register(NRF24.FEATURE,
                                self.read_register(NRF24.FEATURE) | _BV(NRF24.EN_ACK_PAY) | _BV(NRF24.EN_DPL))

        # Enable dynamic payload on pipes 0 & 1
        self.write_register(NRF24.DYNPD, self.read_register(NRF24.DYNPD) | _BV(NRF24.DPL_P1) | _BV(NRF24.DPL_P0))

    def writeAckPayload(self, pipe, buf, buf_len):
        txbuffer = [NRF24.W_ACK_PAYLOAD | ( pipe & 0x7 )]

        max_payload_size = 32
        data_len = min(buf_len, max_payload_size)
        txbuffer.extend(buf[0:data_len])

        self.spidev.xfer2(txbuffer)

    def isAckPayloadAvailable(self):
        result = self.ack_payload_available
        self.ack_payload_available = False
        return result

    def isPVariant(self):
        return self.p_variant

    def setAutoAck(self, enable):
        if enable:
            self.write_register(NRF24.EN_AA, int('111111',2))
        else:
            self.write_register(NRF24.EN_AA, 0)

    def setAutoAckPipe(self, pipe, enable):
        if pipe <= 6:
            en_aa = self.read_register(NRF24.EN_AA)
            if enable:
                en_aa |= _BV(pipe)
            else:
                en_aa &= ~_BV(pipe)

            self.write_register(NRF24.EN_AA, en_aa)

    def testCarrier(self):
        return self.read_register(NRF24.CD) & 1

    def testRPD(self):
        return self.read_register(NRF24.RPD) & 1

    def setPALevel(self, level):
        setup = self.read_register(NRF24.RF_SETUP)
        setup &= ~( _BV(NRF24.RF_PWR_LOW) | _BV(NRF24.RF_PWR_HIGH))
        # switch uses RAM (evil!)
        if level == NRF24.PA_MAX:
            setup |= (_BV(NRF24.RF_PWR_LOW) | _BV(NRF24.RF_PWR_HIGH))
        elif level == NRF24.PA_HIGH:
            setup |= _BV(NRF24.RF_PWR_HIGH)
        elif level == NRF24.PA_LOW:
            setup |= _BV(NRF24.RF_PWR_LOW)
        elif level == NRF24.PA_MIN:
            nop = 0
        elif level == NRF24.PA_ERROR:
            # On error, go to maximum PA
            setup |= (_BV(NRF24.RF_PWR_LOW) | _BV(NRF24.RF_PWR_HIGH))

        self.write_register(NRF24.RF_SETUP, setup)


    def getPALevel(self):
        power = self.read_register(NRF24.RF_SETUP) & (_BV(NRF24.RF_PWR_LOW) | _BV(NRF24.RF_PWR_HIGH))

        if power == (_BV(NRF24.RF_PWR_LOW) | _BV(NRF24.RF_PWR_HIGH)):
            return NRF24.PA_MAX
        elif power == _BV(NRF24.RF_PWR_HIGH):
            return NRF24.PA_HIGH
        elif power == _BV(NRF24.RF_PWR_LOW):
            return NRF24.PA_LOW
        else:
            return NRF24.PA_MIN

    def setDataRate(self, speed):
        result = False
        setup = self.read_register(NRF24.RF_SETUP)

        # HIGH and LOW '00' is 1Mbs - our default
        self.wide_band = False
        setup &= ~(_BV(NRF24.RF_DR_LOW) | _BV(NRF24.RF_DR_HIGH))

        if speed == NRF24.BR_250KBPS:
            # Must set the RF_DR_LOW to 1 RF_DR_HIGH (used to be RF_DR) is already 0
            # Making it '10'.
            self.wide_band = False
            setup |= _BV(NRF24.RF_DR_LOW)
        else:
            # Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
            # Making it '01'
            if speed == NRF24.BR_2MBPS:
                self.wide_band = True
                setup |= _BV(NRF24.RF_DR_HIGH)
            else:
                # 1Mbs
                self.wide_band = False

        self.write_register(NRF24.RF_SETUP, setup)

        # Verify our result
        if self.read_register(NRF24.RF_SETUP) == setup:
            result = True
        else:
            self.wide_band = False
        return result

    def getDataRate(self):
        dr = self.read_register(NRF24.RF_SETUP) & (_BV(NRF24.RF_DR_LOW) | _BV(NRF24.RF_DR_HIGH))
        # Order matters in our case below
        if dr == _BV(NRF24.RF_DR_LOW):
            # '10' = 250KBPS
            return NRF24.BR_250KBPS
        elif dr == _BV(NRF24.RF_DR_HIGH):
            # '01' = 2MBPS
            return NRF24.BR_2MBPS
        else:
            # '00' = 1MBPS
            return NRF24.BR_1MBPS


    def setCRCLength(self, length):
        config = self.read_register(NRF24.CONFIG) & ~( _BV(NRF24.CRC_16) | _BV(NRF24.CRC_ENABLED))

        if length == NRF24.CRC_DISABLED:
            # Do nothing, we turned it off above.
            self.write_register(NRF24.CONFIG, config)
            return
        elif length == NRF24.CRC_8:
            config |= _BV(NRF24.CRC_ENABLED)
            config |= _BV(NRF24.CRC_8)
        else:
            config |= _BV(NRF24.CRC_ENABLED)
            config |= _BV(NRF24.CRC_16)

        self.write_register(NRF24.CONFIG, config)

    def getCRCLength(self):
        result = NRF24.CRC_DISABLED
        config = self.read_register(NRF24.CONFIG) & ( _BV(NRF24.CRCO) | _BV(NRF24.EN_CRC))

        if config & _BV(NRF24.EN_CRC):
            if config & _BV(NRF24.CRCO):
                result = NRF24.CRC_16
            else:
                result = NRF24.CRC_8

        return result

    def disableCRC(self):
        disable = self.read_register(NRF24.CONFIG) & ~_BV(NRF24.EN_CRC)
        self.write_register(NRF24.CONFIG, disable)

    def setRetries(self, delay, count):
        self.write_register(NRF24.SETUP_RETR, (delay & 0xf) << NRF24.ARD | (count & 0xf) << NRF24.ARC)


    def getRetries(self):
        return self.read_register(NRF24.SETUP_RETR)

    def getMaxTimeout(self):
        retries = self.getRetries()
        return ((250+(250*((retries& 0xf0)>>4 ))) * (retries & 0x0f)) / 1000000.0
