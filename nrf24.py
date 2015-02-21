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


try:
    #For BBBB
    import Adafruit_BBIO.GPIO as GPIO
except ImportError:
    try:
        # For Raspberry Pi
        import RPi.GPIO as GPIO
    except ImportError:
        raise ImportError('Neither RPi.GPIO nor Adafruit_BBIO.GPIO module found.')

# Use a monotonic clock if available to avoid unwanted side effects from clock
# changes
try:
    from time import monotonic
except ImportError:
    from time import time as monotonic

import spidev
import time
import sys
if sys.version > '3':
    long = int


class NRF24(object):
    MAX_CHANNEL = 127
    MAX_PAYLOAD_SIZE = 32

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
    MASK_RX_DR = 0x40
    MASK_TX_DS = 0x20
    MASK_MAX_RT = 0x10
    EN_CRC = 0x08
    CRCO = 0x04
    PWR_UP = 0x02
    PRIM_RX = 0x01
    PLL_LOCK = 0x10
    RX_DR = 0x40
    TX_DS = 0x20
    MAX_RT = 0x10
    TX_FULL = 0x01
    TX_REUSE = 6
    FIFO_FULL = 5
    TX_EMPTY = 4
    RX_FULL = 1
    RX_EMPTY = 0
    EN_DPL = 0x04
    EN_ACK_PAY = 0x02
    EN_DYN_ACK = 0x01

    # Shift counts
    ARD = 4
    ARC = 0
    PLOS_CNT = 4
    ARC_CNT = 0
    RX_P_NO = 1

    # Masks
    RX_P_NO_MASK = 0x0E

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
    RF_DR_LOW = 0x20
    RF_DR_HIGH = 0x08

    RF_PWR_LOW = 0x02
    RF_PWR_HIGH = 0x04

    model_e_str_P = ["nRF24L01", "nRF24l01+"]


    @staticmethod
    def _to_8b_list(data):
        """Convert an arbitray iteratable or single int to a list of ints
            where each int is smaller than 256."""
        if isinstance(data, (int, long)):
            data = [data]
        elif isinstance(data, str):
            data = [ord(x) for x in data]
        else:
            data = [int(x) for x in data]

        for byte in data:
            if byte < 0 or byte > 255:
                raise RuntimeError("Value %d is larger than 8 bits" % byte)
        return data


    def __init__(self, major, minor, ce_pin, irq_pin=None):
        # Init spi
        self.spidev = spidev.SpiDev()
        self.spidev.open(major, minor)
        self.spidev.bits_per_word = 8
        self.spidev.cshigh = False
        self.spidev.loop = False
        self.spidev.lsbfirst = False
        try:
            self.spidev.max_speed_hz = 10000000 #Maximum supported by NRF24L01+
        except IOError:
            pass # Hardware does not support this speed, use default speed instead
        self.spidev.mode = 0
        self.spidev.threewire = False


        self.ce_pin = ce_pin
        self.irq_pin = irq_pin

        GPIO.setup(self.ce_pin, GPIO.OUT)
        if self.irq_pin is not None:
            GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.reset()
        self.power_up()

        # Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to
        # make testing a little easier
        # WARNING: If this is ever lowered, either 250KBS mode with AA is
        # broken or maximum packet sizes must never be used. See documentation
        # for a more complete explanation.
        self.delay = None # *< Delay between retransmissions
        self.retries_cached = None
        self.set_retries(4000, 15)

        # Restore our default PA level
        self.dbm = 0

        # Determine if this is a p or non-p RF24 module and then
        # reset our data rate back to default value. This works
        # because a non-P variant won't allow the data rate to
        # be set to 250Kbps.
        try:
            self.data_rate = 250
            self.p_variant = True
        except RuntimeError:
            self.p_variant = False

        # Don't set defaults for critical values to avoid users relying on
        # these values
        self.data_rate_cached = None
        self.channel_cached = None
        self.crc_length_cached = None
        self.payload_size_cached = None

        # Disable dynamic payloads, to match dynamic_payloads_enabled setting
        self.write_register(NRF24.DYNPD, 0)
        self.dynamic_payloads_enabled = False  # *< Whether dynamic payloads are enabled.

        # Reset current status
        # Notice reset and flush is the last thing we do
        self.clear_irq_flags()

        # Flush buffers
        self.flush_rx()
        self.flush_tx()


        #self.ack_payload_available = False  # *< Whether there is an ack payload waiting
        #self.ack_payload_length = 5  # *< Dynamic size of pending ack payload.
        self.pipe0_reading_address = None  # *< Last address set on pipe 0 for reading.
        self.last_error = None

    def end(self):
        self.set_ce(0)
        self.spidev.close()
        self.spidev = None

    def reset(self):
        """ Make sure the NRF is in the same state as after power up
            to avoid problems resulting from left over configuration
            from other programs."""
        self.set_ce(0)
        reset_values = {0: 0x08, 1: 0x3F, 2:0x03, 3:0x03, 4:0x03, 5:0x02, 6:0x0E,
                        0x0a: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x0b: [0xc2, 0xc2, 0xc2, 0xc2, 0xc2],
                        0x0c: 0xc3, 0x0d:0xc4, 0x0e:0xc5, 0x0f:0xc6,
                        0x10: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x11: 0, 0x12:0, 0x13:0, 0x14: 0, 0x15: 0, 0x16: 0,
                        0x1c: 0, 0x1d:0}
        for reg, value in reset_values.items():
            self.write_register(reg, value)
        self.flush_rx()
        self.flush_tx()


    def set_ce(self, level):
        GPIO.output(self.ce_pin, level)

    def irq_wait(self, timeout=30000):
        # TODO: A race condition may occur here. => wait for level?
        if GPIO.input(self.irq_pin) == 0:  # Pin is already down. Packet is waiting?
            return True

        try:
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING, timeout) == 1
        except TypeError: # Timeout parameter not supported
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING) == 1
        except AttributeError:
            raise RuntimeError("GPIO lib does not support wait_for_edge()")

    def read_register(self, reg, length=1):
        """Read a register. Returns value as int or list depending on length."""
        buf = [NRF24.R_REGISTER | (NRF24.REGISTER_MASK & reg)]
        buf += [NRF24.NOP] * max(1, length)

        resp = self.spidev.xfer2(buf)
        if length == 1:
            return resp[1]

        return resp[1:]

    def write_register(self, reg, value):
        """ Write register value """
        buf = [NRF24.W_REGISTER | (NRF24.REGISTER_MASK & reg)]
        buf += self._to_8b_list(value)
        self.spidev.xfer2(buf)

    def write_payload(self, buf):
        """ Writes data to the payload register, automatically padding it
            to match the required length. Returns the number of bytes
            actually written. """
        buf = self._to_8b_list(buf)
        if self.dynamic_payloads_enabled:
            if len(buf) > self.MAX_PAYLOAD_SIZE:
                raise RuntimeError("Dynamic payload is larger than the "+
                    "maximum size.")
            blank_len = 0
        else:
            if len(buf) > self.payload_size:
                raise RuntimeError("Payload is larger than the fixed payload" +
                    "size (%d vs. %d bytes)" % (len(buf), self.payload_size))
            blank_len = self.payload_size - len(buf)

        txbuffer = [NRF24.W_TX_PAYLOAD] + buf + ([0x00] * blank_len)

        self.spidev.xfer2(txbuffer)
        return len(txbuffer) - 1

    def read(self):
        if self.dynamic_payloads_enabled:
            buf_len = self.dynamic_payload_size()
            if buf_len > 32:
                print("Warning: Invalid packet length", buf_len)
                self.flush_rx()
                return None
        else:
            buf_len = self.payload_size

        payload = self.spidev.xfer2([NRF24.R_RX_PAYLOAD] + [NRF24.NOP] * buf_len)
        return payload[1:buf_len + 1]

    def clear_irq_flags(self):
        self.write_register(NRF24.STATUS, NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT)

    def flush_rx(self):
        return self.spidev.xfer2([NRF24.FLUSH_RX])[0]

    def flush_tx(self):
        return self.spidev.xfer2([NRF24.FLUSH_TX])[0]

    def status(self):
        return self.spidev.xfer2([NRF24.NOP])[0]

    @staticmethod
    def print_single_status_line(name, value):
        print("{0:<16}= {1}".format(name, value))

    def print_status(self, status=None):
        if status is None:
            status = self.status()
        status_str = "0x{0:02x}: RX_DR={1:x} TX_DS={2:x} MAX_RT={3:x} RX_P_NO={4:x} TX_FULL={5:x}".format(
            status,
            1 if status & NRF24.RX_DR else 0,
            1 if status & NRF24.TX_DS else 0,
            1 if status & NRF24.MAX_RT else 0,
            ((status >> NRF24.RX_P_NO) & 0b111),
            1 if status & NRF24.TX_FULL else 0)

        self.print_single_status_line("STATUS", status_str)

    def print_observe_tx(self, value=None):
        if value is None:
            value = self.read_register(NRF24.OBSERVE_TX)
        tx_str = "0x{0:02x}: POLS_CNT={1:x} ARC_CNT={2:x}".format(
            value,
            (value >> NRF24.PLOS_CNT) & 0b1111,
            (value >> NRF24.ARC_CNT) & 0b1111)

        self.print_single_status_line("OBSERVE_TX", tx_str)

    def print_byte_register(self, name, reg, qty=1):
        registers = ["0x{:0>2x}".format(self.read_register(reg+r)) for r in range(0, qty)]
        self.print_single_status_line(name, " ".join(registers))

    def print_address_register(self, name, reg, qty=1):
        address_registers = ["0x{0:>02x}{1:>02x}{2:>02x}{3:>02x}{4:>02x}".format(
            *self.read_register(reg+r, 5))
            for r in range(qty)]

        self.print_single_status_line(name, " ".join(address_registers))

    def print_details(self):
        self.print_byte_register("CONFIG", NRF24.CONFIG)
        self.print_byte_register("EN_AA", NRF24.EN_AA)
        self.print_byte_register("EN_RXADDR", NRF24.EN_RXADDR)
        self.print_byte_register("SETUP_AW", NRF24.SETUP_AW)
        self.print_byte_register("SETUP_RETR", NRF24.SETUP_RETR)
        self.print_byte_register("RF_CH", NRF24.RF_CH)
        self.print_byte_register("RF_SETUP", NRF24.RF_SETUP)
        self.print_status()
        self.print_observe_tx()
        self.print_address_register("RX_ADDR_P0-1", NRF24.RX_ADDR_P0, 2)
        self.print_byte_register("RX_ADDR_P2-5", NRF24.RX_ADDR_P2, 4)
        self.print_address_register("TX_ADDR", NRF24.TX_ADDR)
        self.print_byte_register("RX_PW_P0-5", NRF24.RX_PW_P0, 6)
        self.print_byte_register("FIFO_STATUS", NRF24.FIFO_STATUS)
        self.print_byte_register("DYNPD/FEATURE", NRF24.DYNPD, 2)

        self.print_single_status_line("Data Rate", '%d kbps' % self.data_rate)
        self.print_single_status_line("Model", NRF24.model_e_str_P[self.p_variant])
        self.print_single_status_line("CRC Length", '%d Bytes' % self.crc_length)
        self.print_single_status_line("PA Power", '%d dBm' % self.dbm)

    def start_listening(self):
        self._check_settings()
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) |
                            NRF24.PWR_UP | NRF24.PRIM_RX)
        self.clear_irq_flags()

        # Restore the pipe0 address, if exists
        if self.pipe0_reading_address:
            self.write_register(self.RX_ADDR_P0, self.pipe0_reading_address)
        else:
            # Disable pipe 0 (only used for auto-ack)
            self.write_register(NRF24.EN_RXADDR, self.read_register(NRF24.EN_RXADDR) & 0xFE)

        # Go!
        self.set_ce(1)
        # There is no need to wait for the radio to come up, as no bad things
        # happen during this time

    def stop_listening(self):
        self._check_settings()
        self.set_ce(0)
        self.flush_tx()
        self.flush_rx()
        self.clear_irq_flags()

        # Enable TX
        self.write_register(NRF24.CONFIG,
                            (self.read_register(NRF24.CONFIG) | NRF24.PWR_UP) & ~NRF24.PRIM_RX)

        # Enable pipe 0 for auto-ack
        self.write_register(NRF24.EN_RXADDR, self.read_register(NRF24.EN_RXADDR) | 1)

    def power_down(self):
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) & ~NRF24.PWR_UP)

    def power_up(self):
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) | NRF24.PWR_UP)
        time.sleep(150e-6)

    def write(self, buf):
        """ Send a data packet.
            Returns true if the packet was transmitted sucessfully."""
        self.last_error = None
        length = self.write_payload(buf)
        self.set_ce(1)
        time.sleep(10e-6)
        self.set_ce(0)

        # 57 bits preamble/address, data rate is kbit/s => *1000
        packet_time = float((length + self.crc_length_cached) * 8 + 57)/(self.data_rate_cached*1000)

        #timeout = self.retries_cached * (packet_time + self.delay*1e-6)
        # Actually the measure timeouts are always a bit longer than the calculated
        # ones. So just stick with a sane value with a bit of safety margin
        # 2M: 0.1s
        # 1M: 0.2s
        # 250k: 0.8s
        timeout = monotonic() + 200. / self.data_rate_cached
        while monotonic() < timeout:
            time.sleep(packet_time)
            status = self.status()
            if status & NRF24.TX_DS:
                return True
            if status & NRF24.MAX_RT:
                self.last_error = 'MAX_RT'
                break
            time.sleep(self.delay*1e-6)
        if self.last_error is None:
            self.last_error = 'TIMEOUT'
        self.flush_tx() # Avoid leaving the payload in tx fifo
        return False

    def dynamic_payload_size(self):
        """ Read the size of the most recent packet in fifo. """
        return self.spidev.xfer2([NRF24.R_RX_PL_WID, NRF24.NOP])[1]

    def available(self, pipe_num=None, irq_wait=False, irq_timeout=30000):
        status = self.status()
        result = False

        # Sometimes the radio specifies that there is data in one pipe but
        # doesn't set the RX flag...
        if status & NRF24.RX_DR or (status & NRF24.RX_P_NO_MASK != NRF24.RX_P_NO_MASK):
            result = True
        else:
            if irq_wait:  # Will use IRQ wait
                if self.irq_wait(irq_timeout):  # Do we have a packet?
                    status = self.status()  # Seems like we do!
                    if status & NRF24.RX_DR or (status & NRF24.RX_P_NO_MASK != NRF24.RX_P_NO_MASK):
                        result = True

        del pipe_num[:]
        if result and pipe_num is not None:
            # If the caller wants the pipe number, include that
            pipe_num.append((status & NRF24.RX_P_NO_MASK) >> NRF24.RX_P_NO)

        # Clear the status bit

        # ??? Should this REALLY be cleared now?  Or wait until we
        # actually READ the payload?
        self.write_register(NRF24.STATUS, NRF24.RX_DR)

        # Handle ack payload receipt
        if status & NRF24.TX_DS:
            self.write_register(NRF24.STATUS, NRF24.TX_DS)

        return result

    def open_writing_pipe(self, value):
        self.write_register(NRF24.RX_ADDR_P0, value)
        self.write_register(NRF24.TX_ADDR, value)
        if not self.dynamic_payloads_enabled:
            self.write_register(NRF24.RX_PW_P0, self.payload_size_cached)

    def open_reading_pipe(self, pipe, address):
        if pipe >= 6:
            raise RuntimeError("Invalid pipe number")
        if (pipe >= 2 and len(address) > 1) or len(address) > 5:
            raise RuntimeError("Invalid adress length")
        # If this is pipe 0, cache the address.  This is needed because
        # openWritingPipe() will overwrite the pipe 0 address, so
        # startListening() will have to restore it.
        if pipe == 0:
            self.pipe0_reading_address = address
        self.write_register(NRF24.RX_ADDR_P0 + pipe, address)

        if not self.dynamic_payloads_enabled:
            self.write_register(NRF24.RX_PW_P0 + pipe, self.payload_size_cached)

        # Note it would be more efficient to set all of the bits for all open
        # pipes at once.  However, I thought it would make the calling code
        # more simple to do it this way.
        self.write_register(NRF24.EN_RXADDR,
                            self.read_register(NRF24.EN_RXADDR) | (1<<pipe))

    def close_reading_pipe(self, pipe):
        self.write_register(NRF24.EN_RXADDR,
                            self.read_register(NRF24.EN_RXADDR) & ~(1<<pipe))

    def toggle_features(self):
        buf = [NRF24.ACTIVATE, 0x73]
        self.spidev.xfer2(buf)

    def enable_dynamic_payloads(self):
        # Enable dynamic payload throughout the system
        self.write_register(NRF24.FEATURE, self.read_register(NRF24.FEATURE) | NRF24.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self.read_register(NRF24.FEATURE):
            # So enable them and try again
            self.toggle_features()
            self.write_register(NRF24.FEATURE, self.read_register(NRF24.FEATURE) | NRF24.EN_DPL)

        # Enable dynamic payload on all pipes

        # Not sure the use case of only having dynamic payload on certain
        # pipes, so the library does not support it.
        self.write_register(NRF24.DYNPD, self.read_register(NRF24.DYNPD) | 0b00111111)

        self.dynamic_payloads_enabled = True

    def enable_ack_payload(self):
        # enable ack payload and dynamic payload features
        self.write_register(NRF24.FEATURE,
                            self.read_register(NRF24.FEATURE) | NRF24.EN_ACK_PAY | NRF24.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self.read_register(NRF24.FEATURE):
            # So enable them and try again
            self.toggle_features()
            self.write_register(NRF24.FEATURE,
                                self.read_register(NRF24.FEATURE) | NRF24.EN_ACK_PAY | NRF24.EN_DPL)

        # Enable dynamic payload on pipes 0 & 1
        self.write_register(NRF24.DYNPD, self.read_register(NRF24.DYNPD) | 0b11)

    def write_ack_payload(self, pipe, buf, buf_len):
        txbuffer = [NRF24.W_ACK_PAYLOAD | (pipe & 0x7)]

        max_payload_size = 32
        data_len = min(buf_len, max_payload_size)
        txbuffer.extend(buf[0:data_len])

        self.spidev.xfer2(txbuffer)

    def set_auto_ack(self, enable, mask=0x3F):
        if enable:
            self.write_register(NRF24.EN_AA, mask)
        else:
            self.write_register(NRF24.EN_AA, 0)

    def set_auto_ack_pipe(self, pipe, enable):
        if pipe <= 6:
            en_aa = self.read_register(NRF24.EN_AA)
            if enable:
                en_aa |= 1 << pipe
            else:
                en_aa &= ~(1 << pipe)

            self.write_register(NRF24.EN_AA, en_aa)

    def set_address_width(self, width):
        if width >= 2 and width <= 5:
            self.write_register(NRF24.SETUP_AW, width - 2)
        else:
            raise RuntimeError("Invalid address width")

    def channel_active(self):
        return self.read_register(NRF24.CD) & 1

    @property
    def dbm(self):
        power = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH)

        if power == (NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH):
            return 0
        elif power == NRF24.RF_PWR_HIGH:
            return -6
        elif power == NRF24.RF_PWR_LOW:
            return -12
        else:
            return -18

    @dbm.setter
    def dbm(self, level):
        setup = self.read_register(NRF24.RF_SETUP)
        setup &= ~(NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH)
        if level == 0:
            setup |= NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH
        elif level == -6:
            setup |= NRF24.RF_PWR_HIGH
        elif level == -12:
            setup |= NRF24.RF_PWR_LOW
        elif level != -18:
            raise RuntimeError("Invalid power level")

        self.write_register(NRF24.RF_SETUP, setup)

    @property
    def data_rate(self):
        rate = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)
        if rate == NRF24.RF_DR_LOW:
            return 250
        elif rate == NRF24.RF_DR_HIGH:
            return 2000
        else:
            return 1000

    @data_rate.setter
    def data_rate(self, speed):
        setup = self.read_register(NRF24.RF_SETUP) & ~(NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)
        if speed == 250:
            setup |= NRF24.RF_DR_LOW
        elif speed == 1000:
            pass
        elif speed == 2000:
            setup |= NRF24.RF_DR_HIGH
        else:
            raise RuntimeError("Invalid data rate")
        self.data_rate_cached = speed
        self.write_register(NRF24.RF_SETUP, setup)
        if self.read_register(NRF24.RF_SETUP) != setup:
            raise RuntimeError("Datarate not supported")

    @property
    def crc_length(self):
        config = self.read_register(NRF24.CONFIG) & (NRF24.CRCO | NRF24.EN_CRC)
        if config & NRF24.EN_CRC:
            if config & NRF24.CRCO:
                return 2
            else:
                return 1
        return 0

    @crc_length.setter
    def crc_length(self, length):
        config = self.read_register(NRF24.CONFIG) & ~(NRF24.CRCO | NRF24.EN_CRC)
        if length == 0:
            pass
        elif length == 1:
            config |= NRF24.EN_CRC
        elif length == 2:
            config |= NRF24.EN_CRC | NRF24.CRCO
        else:
            raise RuntimeError("Invalid CRC length")
        self.crc_length_cached = length
        self.write_register(NRF24.CONFIG, config)

    @property
    def channel(self):
        return self.read_register(NRF24.RF_CH)

    @channel.setter
    def channel(self, channel):
        if channel < 0 or channel > self.MAX_CHANNEL:
            raise RuntimeError("Channel number out of range")
        self.channel_cached = channel
        self.write_register(NRF24.RF_CH, channel)

    @property
    def payload_size(self):
        return self.payload_size_cached

    @payload_size.setter
    def payload_size(self, size):
        if size < 1 or size > self.MAX_PAYLOAD_SIZE:
            raise RuntimeError("Payload size of of range")
        # Size is set in open_reading_pipe()
        self.payload_size_cached = size



    def set_retries(self, delay, count):
        if delay % 250 != 0:
            raise RuntimeError("Delay can only be specified in increments of 250µs")
        if delay < 250 or delay > 4000:
            raise RuntimeError("Invalid delay")
        if count < 0 or count > 15:
            raise RuntimeError("Invalid count")
        self.write_register(NRF24.SETUP_RETR, ((delay // 250)-1) << NRF24.ARD | count << NRF24.ARC)
        self.delay = delay
        self.retries_cached = count

    def raw_retries(self):
        return self.read_register(NRF24.SETUP_RETR)

    def _check_settings(self):
        if self.data_rate_cached is None:
            raise RuntimeError("No data rate specified")
        if self.crc_length_cached is None:
            raise RuntimeError("No CRC length specified")
        if self.channel_cached is None:
            raise RuntimeError("No channel specified")
