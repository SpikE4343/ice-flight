from sys import platform
from typing import List

from nmigen.hdl.ir import Elaboratable
from nmigen import *
from nmigen.build import *
from nmigen import sim
from abc import ABCMeta, abstractmethod
from enum import Enum


class Fport(Elaboratable):
    """ SPI Master controller:
            LSB, 1 start bit, 1 stop bit
    """

    class FrameType(Enum):
        CONTROL = 0x00
        DOWNLINK = 0x01
        UPLINK = 0x81

    def __init__(self, clk_freq, uart_rx, memory):
        # Signals
        self.fsm = None
        self.ready = Signal()
        self.uart = uart_rx
        self.memory = memory
        self.header = 0x7E
        # TODO: move to memory
        self.headerType = Signal(8)
        self.payloadLength = Signal(8)

    def ports(self) -> List[Signal]:
        return [
            self.ready,
            self.uart.rx.data,
            self.payloadLength,
            self.headerType
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.d.sync += self.uart.rx.accepted.eq(0)

        with m.FSM(reset="Reset") as self.fsm:
            with m.State("FindMarker"):
                with m.If(~self.uart.rx.accepted & self.uart.rx.ready):
                    with m.If(self.uart.rx.data == self.header):
                        m.next = "FindMarkerEnd"
                    m.d.sync += self.uart.rx.accepted.eq(1)

            with m.State("FindMarkerEnd"):
                with m.If(~self.uart.rx.accepted & self.uart.rx.ready):
                    with m.If(self.uart.rx.data == self.header):
                        m.next = "FindMarkerEnd"
                    with m.Else():
                        m.d.sync += [
                            self.payloadLength.eq(self.uart.rx.data),
                            self.uart.rx.accepted.eq(1)
                        ]
                        m.next = "HeaderType"
                    m.d.sync += self.uart.rx.accepted.eq(1)

            with m.State("HeaderType"):
                with m.If(~self.uart.rx.accepted & self.uart.rx.ready):
                    m.d.sync += [
                        self.headerType.eq(self.uart.rx.data),
                        self.uart.rx.accepted.eq(1)
                    ]
                    m.next = "Payload"

            with m.State("Payload"):
                with m.If(~self.uart.rx.accepted & self.uart.rx.ready):
                    m.d.sync += [
                        # self.data.eq(self.uart.rx.data),
                        # TODO: read data into memory based on packet type
                        self.uart.rx.accepted.eq(1)
                    ]
                    with m.If(self.payloadLength > 0):
                        m.d.sync += self.payloadLength.eq(
                            self.payloadLength - 1)
                    with m.Else():
                        m.next = "FrameReady"

            with m.State("CRC"):
                pass

            with m.State("FrameReady"):
                m.d.sync += self.ready.eq(1)
                m.next = "Reset"

            with m.State("Reset"):
                m.d.sync += [
                    self.ready.eq(0),
                    self.payloadLength.eq(0),
                    self.headerType.eq(0)
                ]
                m.next = "FindMarker"

        return m

    def handlePayloadData(self, m: Module):
        with m.Switch(self.headerType):
            with m.Case(Fport.FrameType.CONTROL):
                self.handleControlFrameData(m)
            with m.Case(Fport.FrameType.DOWNLINK):
                pass
            with m.Case(Fport.FrameType.UPLINK):
                pass

    def handleControlFrameData(self, m: Module):
        with m.Switch(self.payloadLength):
            with m.Case(0x19):
                pass
            with m.Case(0x18):
                pass
            with m.Case(0x17):
                pass
            with m.Case(0x16):
                pass
            with m.Case(0x15):
                pass
            with m.Case(0x14):
                pass
            with m.Case(0x13):
                pass
            with m.Case(0x12):
                pass
            with m.Case(0x11):
                pass
            with m.Case(0x10):
                pass
            with m.Case(0x0F):
                pass
            with m.Case(0x0E):
                pass
            with m.Case(0x0D):
                pass
            with m.Case(0x0C):
                pass
            with m.Case(0x0B):
                pass
            with m.Case(0x0A):
                pass
            with m.Case(0x09):
                pass
            with m.Case(0x08):
                pass
            with m.Case(0x07):
                pass
            with m.Case(0x06):
                pass
            with m.Case(0x05):
                pass
            with m.Case(0x04):
                pass
            with m.Case(0x03):
                pass
            with m.Case(0x02):
                pass

            # Flags
            with m.Case(0x01):
                pass

            # RSSI
            with m.Case(0x00):
                pass


def _test_loopback(dut):

    data = readmemory('../../../../captures/rxsr-fport-data-1024.txt')

    def run_test():
        rx = dut.uart.rx
        for d in data:
            # print(d)

            bits = len(rx.data)
            while(bits > 0):
                yield
                bits = bits - 1

                # print("b:", bits)
                timer = rx.clk_per_bit

                while (timer > 0):
                    yield
                    timer = timer - 1
                    # print(" t:", timer)
                    yield
                yield

            yield rx.data.eq(d)
            yield rx.ready.eq(1)

            while(yield rx.accepted) == 0:
                yield

            yield rx.ready.eq(0)

            # yield dut.send.eq(1)
            # yield

            # while(yield dut.done) == 0:
            #     yield

            # assert(yield dut.rx_data) == d
            # yield

            # yield dut.send.eq(0)
            # yield dut.accept.eq(1)
            # yield
            # yield dut.accept.eq(0)
            yield

    return run_test


if __name__ == "__main__":
    import sys
    from nmigen_boards.tinyfpga_bx import TinyFPGABXPlatform

    def readmemory(filename):
        data = []
        with open(filename, 'r') as reader:
            line = ' '
            while line != '':  # The EOF char is an empty string
                line = reader.readline()
                if(line != ''):
                    sp = line.rstrip().split(' ')
                    data = data + sp

        for d in range(len(data)):
            data[d] = int(data[d], base=16)

        return data

    class MockUartRx(Elaboratable):
        def __init__(self, clk_per_bit, width):
            self.data = Signal(width)
            self.signal = Signal()
            self.ready = Signal()
            self.accepted = Signal()
            self.clk_per_bit = clk_per_bit

        def elaborate(self, platform: Platform) -> Module:
            m = Module()
            return m

    class MockUart(Elaboratable):
        def __init__(self, clk_per_bit, width):
            self.rx = MockUartRx(clk_per_bit, width)

        def elaborate(self, platform: Platform) -> Module:
            m = Module()
            m.submodules.rx = self.rx
            return m

    clk_freq = 16e6
    spi_freq = 2e6
    clk_per_bit = int(clk_freq // spi_freq)

    if len(sys.argv) < 2 or sys.argv[1] == "sim":
        m = Module()

        m.submodules.loopback = loopback = MockUart(clk_per_bit, 8)
        m.submodules.fport = fport = Fport(clk_freq, loopback, None)

        sim = sim.Simulator(m)
        sim.add_clock(1.0 / clk_freq)

        sim.add_sync_process(_test_loopback(fport))

        with sim.write_vcd("fport.vcd", "fport.gtkw", traces=fport.ports()):
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
