from machine import Pin
from array import array
import rp2
from rp2 import PIO

SENSOR_COUNT = const(5)
TIMEOUT = const(1024)
_DONE = const(0)
_READ_LINE = const(1)
_state = _DONE
_qtr = None

class QTRSensors:
    """A multi-channel QTR sensor reader using PIO"""
    @rp2.asm_pio(
        out_init=(PIO.OUT_HIGH,) * SENSOR_COUNT,
        autopush=True, # saves push instructions
        push_thresh=SENSOR_COUNT + 16,
        fifo_join=PIO.JOIN_RX
        )
    def counter():
        # Set OSR to 32 bits of 1s for future shifting out to intialize pindirs,
        # y, y again, and x. This requires SENSOR_COUNT + 8 + 10 + SENSOR_COUNT
        # bits (maximum of 32 bits for SENSOR_COUNT = 7).
        mov(osr, invert(null))

        # Set pindirs to 1s to enable output and start charging
        # the capacitor.
        out(pindirs, SENSOR_COUNT)

        # Charge up the capacitors for ~32us.
        # Set Y counter to 255 by pulling another 8 bits from OSR.
        out(y, 8)
        label("charge")
        jmp(y_dec, "charge")

        # Load 1023 (10 bits of 1s) into Y as a counter
        out(y, 10)

        # Initialize X (last pin state) to 1s.
        out(x, SENSOR_COUNT)

        # Set pins back to inputs by writing 0s to pindirs.
        mov(osr, null)
        out(pindirs, SENSOR_COUNT)

        # loop is 8 instructions long = 1us
        label("loop")

        # read pins into ISR
        in_(pins, SENSOR_COUNT)

        # save y in OSR
        mov(osr, y)

        # compare x to ISR
        mov(y, isr) # new value -> y
        jmp(x_not_y, "change")

        # discard the pin values and reset shift counter
        mov(isr, null)
        jmp("decrement")

        # a pin changed!
        label("change")
        mov(x, y) # save new pins

        # save and write data
        # 7 pins are in low bits of ISR
        in_(osr, 16) # time

        label("decrement")
        mov(y, osr) # restore y
        jmp(y_dec, "loop")

        # END OF PROGRAM
        label("finish")

        # Send 0xFFFFFFFF to tell the CPU we are done.
        in_(y, 32)

        wrap_target()
        nop()
        wrap()

    def __init__(self, id, pin1):
        for i in range(SENSOR_COUNT):
            Pin(pin1+i, Pin.IN, pull=None)

        p = Pin(pin1, Pin.OUT, value=1)
        for i in range(1, SENSOR_COUNT):
            Pin(pin1+i, Pin.OUT, value=1)

        self.sm = rp2.StateMachine(id, self.counter, freq=8000000, in_base=p, out_base=p)
        self.data_line = array('H', [0] * 5)

    def run(self):
        while self.sm.rx_fifo():
            self.sm.get()
        self.sm.restart()
        self.sm.active(1)

    @micropython.viper
    def read_line(self):
        last_states = uint(0x7f0000)
        data = ptr16(self.data_line)
        for i in range(5):
            data[i] = TIMEOUT

        sm = self.sm
        while True:  # TODO: TIMEOUT?
            val = uint(sm.get())
            if(val == uint(0xffffffff)):
                break
            new_zeros = last_states ^ val
            if new_zeros & 0x10000:
                data[4] = TIMEOUT - val
            if new_zeros & 0x20000:
                data[3] = TIMEOUT - val
            if new_zeros & 0x40000:
                data[2] = TIMEOUT - val
            if new_zeros & 0x80000:
                data[1] = TIMEOUT - val
            if new_zeros & 0x100000:
                data[0] = TIMEOUT - val
            last_states = val
        return self.data_line

class _IRSensors():
    def __init__(self):
        self.ir_down = Pin(26, Pin.IN)

        global _qtr
        if not _qtr:
            _qtr = QTRSensors(4, 18)
        self.qtr = _qtr

        self.reset_calibration()

class LineSensors(_IRSensors):
    def _state(self):
        # for testing
        return _state

    def reset_calibration(self):
        self.cal_min = array('H', [1025] * 5)
        self.cal_max = array('H', [0] * 5)

    def calibrate(self):
        tmp_min = array('H', [1025] * 5)
        tmp_max = array('H', [0] * 5)

        # do 10 measurements
        for trials in range(10):
            data = self.read()
            for i in range(5):
                tmp_min[i] = min(data[i], tmp_min[i])
                tmp_max[i] = max(data[i], tmp_max[i])

        # update data only if ALL data beyond one of the limits
        for i in range(5):
            self.cal_max[i] = max(tmp_min[i], self.cal_max[i])
            self.cal_min[i] = min(tmp_max[i], self.cal_min[i])

    def start_read(self, emitters_on=True):
        global _state
        if emitters_on: self.ir_down.init(Pin.OUT, value=1)
        _state = _READ_LINE
        self.qtr.run()

    @micropython.viper
    def read(self):
        global _state
        if uint(_state) != uint(_READ_LINE):
            self.start_read()
        data = self.qtr.read_line()

        self.ir_down.init(Pin.IN)
        _state = _DONE
        return data

    @micropython.viper
    def read_calibrated(self):
        data = self.read()
        d = ptr16(data)
        cal_min = ptr16(self.cal_min)
        cal_max = ptr16(self.cal_max)
        for i in range(5):
            if cal_min[i] >= cal_max[i] or d[i] < cal_min[i]:
                d[i] = 0
            elif d[i] > cal_max[i]:
                d[i]= 1000
            else:
               d[i] = (d[i] - cal_min[i])*1000 // (cal_max[i] - cal_min[i])
        return data
