# Original © 2016 Scott Shawcroft for Adafruit Industries, MIT License
# Ported to MicroPython on 6.Feb.2024 by blobbybilb & BvngeeCord for Albany High School's Electronics Workshop class
# (Unfortunately includes additional buffer allocations to replace CircuitPython C library calls)

import time
from machine import I2C

try:
    from typing import Optional, Type
    from types import TracebackType    
except ImportError:
    pass

class I2CDevice:
    """
    Represents a single I2C device, and provides a layer over default MicroPython libs compatible with `adafruit_bus_device.i2c_device`.
    
    :param ~busio.I2C i2c: machine.I2C object for the device
    :param int device_address: The 7 bit device address
    :param bool probe: Probe for the device upon object creation, default is true
    """

    def __init__(self, i2c: I2C, device_address: int, probe: bool = True) -> None:
        self.i2c = i2c
        self.device_address = device_address
        if probe: self.__probe_for_device()

    def readinto(
        self, buf, *, start: int = 0, end: Optional[int] = None
    ) -> None:
        """
        Read into `buf` from the device. The number of bytes read will be the
        length of `buf`.
        
        :param buffer: buffer to write into
        :param int start: Index to start writing at
        :param int end: Index to write up to but not include; `len(buf)` by default
        """

        end = len(buf) if end == None else end
        x = buf[start:end] # not magic, this is to modify buf rather than make a copy and read into copy
        self.i2c.readfrom_into(self.device_address, x)
        buf[start:end] = x

    def write(
        self, buf, *, start: int = 0, end: Optional[int] = None
    ) -> None:
        """
        Write the bytes from ``buf` to the device, then transmit a stop
        bit.

        :param buff: buffer containing the bytes to write
        :param int start: Index to start writing from
        :param int end: Index to read up to but not include; `len(buf)` by default
        """
        end = len(buf) if end == None else end
        self.i2c.writeto(self.device_address, buf[start:end])
            
    def write_then_readinto(
        self,
        out_buffer,
        in_buffer,
        *,
        out_start: int = 0,
        out_end: Optional[int] = None,
        in_start: int = 0,
        in_end: Optional[int] = None
    ) -> None:
        """
        Write the bytes from ``out_buffer`` to the device, then immediately
        reads into ``in_buffer`` from the device. The number of bytes read
        will be the length of ``in_buffer``.

        If ``out_start`` or ``out_end`` is provided, then the output buffer
        will be sliced as if ``out_buffer[out_start:out_end]``. This will
        not cause an allocation like ``buffer[out_start:out_end]`` will so
        it saves memory.

        If ``in_start`` or ``in_end`` is provided, then the input buffer
        will be sliced as if ``in_buffer[in_start:in_end]``. This will not
        cause an allocation like ``in_buffer[in_start:in_end]`` will so
        it saves memory.

        :param ~ReadableBuffer out_buffer: buffer containing the bytes to write
        :param ~WriteableBuffer in_buffer: buffer containing the bytes to read into
        :param int out_start: Index to start writing from
        :param int out_end: Index to read up to but not include; if None, use ``len(out_buffer)``
        :param int in_start: Index to start writing at
        :param int in_end: Index to write up to but not include; if None, use ``len(in_buffer)``
        """
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)

        self.write(out_buffer, start=out_start, end=out_end)
        self.readinto(in_buffer, start=in_start, end=in_end)

    # def __enter__(self) -> I2CDevice:
    def __enter__(self):
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[type]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType], #type:ignore
    ) -> bool:
        return False

    def __probe_for_device(self) -> None:
        """
        Try to read a byte from an address,
        if you get an OSError it means the device is not there
        or that the device does not support these means of probing
        """
        try:
            self.i2c.writeto(self.device_address, b"")
        except OSError:
            # some OS's dont like writing an empty bytesting...
            # Retry by reading a byte
            try:
                self.i2c.readfrom_into(self.device_address, bytearray(1))
            except OSError:
                # pylint: disable=raise-missing-from
                raise ValueError("No I2C device at address: 0x%x" % self.device_address)
                # pylint: enable=raise-missing-from