# ---------------------------------------------------------------------------
# Bit-banged SWD master for MicroPython on RP2040.
#
# Purpose: give the S57B an SWD host using the Pico that is ALREADY on the
# bench and ALREADY wired to it, instead of buying a probe. Milestone 1 here is
# just READ THE IDCODE -- that alone proves wiring, levels and the ARM debug
# port, and costs two jumper wires.
#
# Wiring (Pico -> S57B header P7):
#     GP6  -> SWCLK  (PA14)
#     GP7  -> SWDIO  (PA13)
#     GND  -> GND        <- already present
# Both are 3V3. The S57B is a 3V3 part, so no level shifting.
#
# Expected STM32F103 (Cortex-M3) DP IDCODE: 0x1BA01477
# ---------------------------------------------------------------------------
from machine import Pin
import time

SWCLK_PIN = 6
SWDIO_PIN = 7


class SWD:
    def __init__(self, swclk=SWCLK_PIN, swdio=SWDIO_PIN):
        self.clk = Pin(swclk, Pin.OUT, value=0)
        self._io_num = swdio
        self.io = Pin(swdio, Pin.OUT, value=1)
        self._out = True

    # --- lowest level ------------------------------------------------------
    def _drive(self, on):
        if on != self._out:
            self.io = Pin(self._io_num, Pin.OUT) if on else Pin(self._io_num, Pin.IN, Pin.PULL_UP)
            self._out = on

    def _wr_bit(self, b):
        self.io.value(b)
        self.clk.value(0)
        self.clk.value(1)

    def _rd_bit(self):
        self.clk.value(0)
        b = self.io.value()
        self.clk.value(1)
        return b

    def _wr(self, val, n):
        for _ in range(n):
            self._wr_bit(val & 1)
            val >>= 1

    def _rd(self, n):
        v = 0
        for i in range(n):
            if self._rd_bit():
                v |= 1 << i
        return v

    def _turn(self):
        # one turnaround clock with the line released
        self.clk.value(0)
        self.clk.value(1)

    # --- link bring-up -----------------------------------------------------
    def reset(self):
        """Line reset -> JTAG-to-SWD -> line reset -> idle."""
        self._drive(True)
        self._wr(0xFFFFFFFF, 32)       # >=50 clocks high
        self._wr(0xFFFFFFFF, 32)
        self._wr(0xE79E, 16)           # JTAG-to-SWD select, LSB first
        self._wr(0xFFFFFFFF, 32)
        self._wr(0xFFFFFFFF, 32)
        self._wr(0x00000000, 8)        # idle

    # --- packets -----------------------------------------------------------
    @staticmethod
    def _request(ap, rnw, addr):
        a2 = (addr >> 2) & 1
        a3 = (addr >> 3) & 1
        par = (ap ^ rnw ^ a2 ^ a3) & 1
        # Start=1, APnDP, RnW, A2, A3, parity, Stop=0, Park=1  (sent LSB first)
        return (1 | (ap << 1) | (rnw << 2) | (a2 << 3) | (a3 << 4)
                  | (par << 5) | (0 << 6) | (1 << 7))

    def read_reg(self, addr, ap=False):
        self._drive(True)
        self._wr(self._request(1 if ap else 0, 1, addr), 8)
        self._drive(False)
        self._turn()
        ack = self._rd(3)
        if ack != 1:
            self._turn()
            self._drive(True)
            raise OSError("SWD read ACK=%d (1=OK 2=WAIT 4=FAULT)" % ack)
        val = self._rd(32)
        par = self._rd_bit()
        self._turn()
        self._drive(True)
        self._wr(0, 8)                 # idle
        if (bin(val).count("1") & 1) != par:
            raise OSError("SWD read parity error")
        return val

    def write_reg(self, addr, val, ap=False):
        self._drive(True)
        self._wr(self._request(1 if ap else 0, 0, addr), 8)
        self._drive(False)
        self._turn()
        ack = self._rd(3)
        self._turn()
        self._drive(True)
        if ack != 1:
            raise OSError("SWD write ACK=%d (1=OK 2=WAIT 4=FAULT)" % ack)
        self._wr(val, 32)
        self._wr_bit(bin(val).count("1") & 1)
        self._wr(0, 8)                 # idle

    def idcode(self):
        self.reset()
        return self.read_reg(0x00)

    # --- debug port bring-up + memory access -------------------------------
    def connect(self):
        """Line reset, power up the debug domain, point AP0 at memory."""
        idc = self.idcode()
        self.write_reg(0x00, 0x1E)            # ABORT: clear sticky errors
        self.write_reg(0x08, 0x00000000)      # SELECT: AP0, bank 0
        self.write_reg(0x04, 0x50000000)      # CTRL/STAT: CSYSPWRUPREQ|CDBGPWRUPREQ
        st = 0
        for _ in range(100):
            st = self.read_reg(0x04)
            if (st & 0xF0000000) == 0xF0000000:
                break
        else:
            raise OSError("debug power-up timeout, CTRL/STAT=0x%08X" % st)
        # CSW: 32-bit transfers, auto-increment single
        self.write_reg(0x00, 0x23000052, ap=True)
        return idc

    def read_mem(self, addr):
        self.write_reg(0x04, addr, ap=True)   # TAR
        self.read_reg(0x0C, ap=True)          # DRW -- pipelined, this one is stale
        return self.read_reg(0x0C)            # RDBUFF holds the real value


def probe():
    """Milestone 1: is the ARM debug port alive?"""
    s = SWD()
    try:
        idc = s.idcode()
    except OSError as e:
        print("FAIL:", e)
        print("  check GP6->SWCLK(PA14), GP7->SWDIO(PA13), GND tied, board powered")
        return None

    print("DP IDCODE = 0x%08X" % idc)
    designer = (idc >> 1) & 0x7FF
    partno = (idc >> 12) & 0xFFFF
    ver = (idc >> 28) & 0xF
    print("  version %d  partno 0x%04X  designer 0x%03X" % (ver, partno, designer))
    if idc == 0x1BA01477:
        print("  => Cortex-M3 SW-DP. This is the STM32F103. LINK IS GOOD.")
    elif idc in (0x0BB11477, 0x0BC11477, 0x2BA01477):
        print("  => a Cortex-M SW-DP, but not the F103 IDCODE. Check the target.")
    elif idc == 0 or idc == 0xFFFFFFFF:
        print("  => dead line. Nothing is driving SWDIO.")
    else:
        print("  => unexpected. Report this value before going further.")
    return idc


def info():
    """Milestone 1b: identify the die and settle the flash-size question."""
    s = SWD()
    try:
        idc = s.connect()
    except OSError as e:
        print("FAIL:", e)
        print("  check GP6->SWCLK(PA14), GP7->SWDIO(PA13), GND tied, board powered")
        return None

    print("DP IDCODE      = 0x%08X %s" % (
        idc, "(Cortex-M3 SW-DP, STM32F103)" if idc == 0x1BA01477 else "(UNEXPECTED)"))

    dbg = s.read_mem(0xE0042000)          # DBGMCU_IDCODE
    dev = dbg & 0xFFF
    rev = (dbg >> 16) & 0xFFFF
    names = {0x410: "F103 medium-density", 0x412: "F103 low-density",
             0x414: "F103 high-density", 0x430: "F103 XL-density"}
    print("DBGMCU_IDCODE  = 0x%08X  DEV_ID 0x%03X %s  REV_ID 0x%04X"
          % (dbg, dev, names.get(dev, "(unknown)"), rev))

    # F_SIZE: 16-bit flash size in KB, at 0x1FFFF7E0 on STM32F1.
    fs = s.read_mem(0x1FFFF7E0) & 0xFFFF
    print("F_SIZE         = %d KB" % fs)
    if fs >= 128:
        print("  => the die really has 128KB. genericSTM32F103CB is safe.")
    elif fs == 64:
        print("  => 64KB as marked. Firmware MUST fit in 64KB.")
    else:
        print("  => odd value; re-read before trusting it.")

    return {"idcode": idc, "dev_id": dev, "rev_id": rev, "flash_kb": fs}
