from sys import platform
from typing import List

from nmigen.hdl.ir import Elaboratable
from nmigen import *
from nmigen.build import *
from nmigen import sim
from abc import ABCMeta, abstractmethod


class Fport(Elaboratable):
    """ SPI Master controller:
            LSB, 1 start bit, 1 stop bit
    """

    def __init__(self, clk_freq, uart_rx: Uart, memory):
        # Signals
        self.fsm = None
        self.ready = Signal()
        self.dataByte = Signal(8)
        self.dataReady = Signal()
        self.uart = uart_rx
        self.memory = memory
        self.header = 0x7E

    def ports(self) -> List[Signal]:
        return [
            self.timerCount,
            self.send,
            self.bit
        ]

    def readUartByte(self):

        m.d.sync += [
            self.dataReady.eq(1)
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        with m.FSM(reset="Reset") as self.fsm:
            with m.State("FindMarker"):
                with m.If(self.uart.rx.ready & self.uart.rx.data == self.header):
                    m.next = "FindMarkerEnd"

            with m.State("FindMarkerEnd"):
                self.readUartByte()

                with m.If(self.dataByte == self.header):
                    m.next = "FindMarkerEnd"
                with m.Else():
                    m.next = "HeaderLength"

            with m.State("HeaderLength"):
                pass

            with m.State("HeaderType"):
                pass

            with m.State("Payload"):
                pass

            with m.State("CRC"):
                pass

            with m.State("FrameReady"):
                pass

            with m.State("Reset"):
                pass

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
