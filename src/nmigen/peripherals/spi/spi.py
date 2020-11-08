from sys import platform
from typing import List

from nmigen.hdl.ir import Elaboratable
from nmigen import *
from nmigen.build import *
from nmigen import sim
from abc import ABCMeta, abstractmethod


class SpiMaster(Elaboratable):
    """ SPI Master controller:
            LSB, 1 start bit, 1 stop bit
    """

    def __init__(self, clk_per_bit, bit_width=8):
        # Signals
        self.timerCount = Signal(8, reset=clk_per_bit)
        self.clk_per_bit = clk_per_bit

        # spi lines
        self.sclk = Signal()
        self.mosi = Signal()
        self.miso = Signal()
        self.cs = Signal()

        # control
        self.rx_data = Signal(8)
        self.tx_data = Signal(8)

        self.send = Signal()
        self.done = Signal()
        self.accept = Signal()
        self.bit = Signal(range(bit_width+1), reset=int(bit_width))
        self.fsm = None

    def ports(self) -> List[Signal]:
        return [
            self.timerCount,
            self.send,
            self.bit
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        with m.FSM(reset="Idle") as self.fsm:
            with m.State("Idle"):
                m.d.sync += [
                    self.timerCount.eq(0),
                    self.mosi.eq(1),
                    self.cs.eq(1),
                    self.bit.eq(self.bit.reset)
                ]
                with m.If(self.send):
                    m.next = "StartBit"
                    m.d.sync += self.timerCount.eq(self.timerCount.reset),

            with m.State("StartBit"):
                with m.If(self.timerCount == 0):
                    m.next = "DataBit"
                with m.Else():
                    m.d.sync += [
                        self.timerCount.eq(self.timerCount - 1),
                        self.mosi.eq(0),
                        self.cs.eq(0)
                    ]

            with m.State("DataBit"):
                # set output
                m.d.sync += self.mosi.eq(
                    self.tx_data.bit_select(self.bit.reset - self.bit - 1, 1))

                with m.If(self.timerCount > 0):
                    m.d.sync += [
                        self.timerCount.eq(self.timerCount - 1),
                        self.sclk.eq(
                            Mux(self.timerCount > self.clk_per_bit // 2, 0, 1))
                    ]

                    with m.If(self.timerCount == self.clk_per_bit // 2):
                        # sample input
                        m.d.sync += self.rx_data.bit_select(
                            self.bit.reset - self.bit - 1, 1).eq(self.miso)
                with m.Elif(self.timerCount == 0):
                    m.d.sync += self.timerCount.eq(self.timerCount.reset)
                    with m.If(self.bit == 0):
                        m.next = "StopBit"
                    with m.Else():
                        m.d.sync += self.bit.eq(self.bit - 1)

            with m.State("StopBit"):
                with m.If(self.timerCount == 0):
                    m.next = "Done"
                    m.d.sync += [
                        self.done.eq(1),
                        self.mosi.eq(1),
                        self.sclk.eq(1),
                        self.cs.eq(1)
                    ]
                with m.Else():
                    m.d.sync += [
                        self.timerCount.eq(self.timerCount - 1),
                        self.mosi.eq(0),
                        self.sclk.eq(1),
                        self.cs.eq(0)
                    ]

            with m.State("Done"):
                with m.If(self.accept):
                    m.next = "Idle"
                    m.d.sync += self.done.eq(0)

        return m


def _test_loopback(dut):
    data = [
        0b10101010, 0x7f, 0xFF, 0x0F, 0xA3
    ]

    def run_test():
        for d in data:
            print(d)

            yield dut.tx_data.eq(d)

            yield dut.send.eq(1)
            yield

            while(yield dut.done) == 0:
                yield

            assert(yield dut.rx_data) == d
            yield

            yield dut.send.eq(0)
            yield dut.accept.eq(1)
            yield
            yield dut.accept.eq(0)
            yield

    return run_test


if __name__ == "__main__":
    import sys
    from nmigen_boards.tinyfpga_bx import TinyFPGABXPlatform

    clk_freq = 16e6
    spi_freq = 2e6
    clk_per_bit = int(clk_freq // spi_freq)

    if len(sys.argv) < 2 or sys.argv[1] == "sim":
        m = Module()

        m.submodules.loopback = loopback = SpiMaster(clk_per_bit, 8)

        m.d.comb += loopback.miso.eq(loopback.mosi)

        sim = sim.Simulator(m)
        sim.add_clock(1.0 / clk_freq)

        sim.add_sync_process(_test_loopback(loopback))

        with sim.write_vcd("spi.vcd", "spi.gtkw", traces=loopback.ports()):
            sim.run()
            # while sim.advance():
            #     input("ENTER to continue...")
    elif sys.argv[1] == "build":
        class Board(TinyFPGABXPlatform):
            p = "p1"

        class SpiTest(Elaboratable):
            def __init__(self):
                pass

            def elaborate(self, platform: Platform) -> Module:
                m = Module()

                clk_freq = int(platform.default_clk_frequency)
                m.submodules.dshot = spi = SpiMaster(clk_per_bit, 8)

                led = platform.request("led").o
                m.d.comb += [
                    led.eq(spi.signals[0])
                ]

                return m

        Board().build(DshotTest())
