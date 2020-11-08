from sys import platform
from typing import List

from nmigen.hdl.ir import Elaboratable
from nmigen import *
from nmigen.build import *
from nmigen import sim
from abc import ABCMeta, abstractmethod


class DShotPacketEncoder(Elaboratable):
    def __init__(self):
        self.command = Signal(11)
        self.telemetryReq = Signal()
        self.packet = Signal(16)

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        self.data = Signal(12)
        self.crc = Signal(4)

        m.d.comb += [
            self.data.eq(Cat(self.telemetryReq, self.command)),
            self.crc.eq(
                self.data.bit_select(11, 4) ^
                self.data.bit_select(7, 4) ^
                self.data.bit_select(3, 4)
            ),
            self.packet.eq(Cat(self.crc, self.data))
        ]
        return m


class DShotPacketDecoder(Elaboratable):
    def __init__(self):
        self.packet = Signal(16)
        self.command = Signal(11)
        self.telemetryReq = Signal()
        self.crc = Signal(4)

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.d.comb += [

            self.command.eq(self.packet.bit_select(15, 12)),

            self.crc.eq(
                self.packet.bit_select(15, 4) ^
                self.packet.bit_select(11, 4) ^
                self.packet.bit_select(7, 4)
            ),
            self.telemetryReq.eq(Cat(self.crc, self.data))
        ]
        return m


class DShotTx(Elaboratable):
    def __init__(self, clk_per_bit):
        self.bit = Signal()
        self.signal = Signal()
        self.send = Signal()
        self._one_count = int(clk_per_bit // 3)
        self._zero_count = int((2 * clk_per_bit) // 3)
        self.timerCount = Signal(8, reset=clk_per_bit)

    def ports(self) -> List[Signal]:
        return [self.timerCount,
                self.signal,
                self.bit,
                self.send,
                ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        print("1: {}, 0: {}", self._one_count, self._zero_count)
        m.d.comb += [
            self.signal.eq(self.send & (self.timerCount > Mux(
                self.bit, self._one_count, self._zero_count)))
        ]

        return m


class DShot(Elaboratable):
    def __init__(self, clk_freq, dshot_freq, channelCount):
        self.clk_per_bit = int(clk_freq // dshot_freq)

        # Signals
        self.timerCount = Signal(8, reset=self.clk_per_bit)
        self.send = Signal()
        self.bit = Signal(5)
        self.signals = Array(Signal() for i in range(channelCount))
        self.commands = Array(DShotPacketEncoder()
                              for i in range(channelCount))
        self.channels = Array(DShotTx(self.clk_per_bit)
                              for i in range(channelCount))
        self.done = Signal()
        self.accept = Signal()
        self.fsm = None

    def ports(self) -> List[Signal]:
        return [
            self.timerCount,
            self.send,
            self.bit
        ]  # + self.commands + self.signals

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        with m.FSM(reset="IDLE") as self.fsm:
            with m.State("IDLE"):
                with m.If(self.send):
                    m.d.sync += self.bit.eq(16)
                    m.next = "SENDING"
                with m.Else():
                    m.next = "IDLE"

            with m.State("SENDING"):
                with m.If(self.timerCount == 0):
                    with m.If(self.bit == 1):
                        m.next = "DONE"
                        m.d.sync += self.done.eq(1)
                    with m.Else():
                        m.d.sync += [
                            self.bit.eq(self.bit - 1),
                            self.timerCount.eq(self.timerCount.reset)
                        ]
                with m.Else():
                    m.d.sync += self.timerCount.eq(self.timerCount - 1)

            with m.State("DONE"):
                with m.If(self.accept):
                    m.next = "RESET"

            with m.State("RESET"):
                with m.If(self.send):
                    m.next = "RESET"
                with m.Else():
                    m.next = "IDLE"
                    m.d.sync += [
                        self.timerCount.eq(0),
                        self.bit.eq(0),
                        self.done.eq(0)
                    ]

        for c in range(len(self.channels)):
            channel = self.channels[c]
            m.submodules['channel_'+str(c)] = channel
            m.submodules['encoder_'+str(c)] = self.commands[c]

            m.d.comb += [
                channel.send.eq(self.send),
                channel.timerCount.eq(self.timerCount),
                self.signals[c].eq(~self.done & channel.signal),
            ]

            m.d.comb += channel.bit.eq(
                self.commands[c].packet.bit_select(self.bit-1, 1))

        return m


def _test_loopback(dut):
    channelCount = 4
    data = [
        0b1010101010001010,
        0b1010101001001010,
        0b1010101000101010,
        0b1010101000010010
    ]

    def run_test():
        # or d in data:
        # print(d)

        for i in range(channelCount):
            print(data[i])
            yield dut.commands[i].command.eq(data[i])

        yield dut.send.eq(1)
        yield

        while(yield dut.done) == 0:
            yield

        yield dut.send.eq(0)
        yield dut.accept.eq(1)
        yield dut.accept.eq(0)

    return run_test


if __name__ == "__main__":
    import sys
    from nmigen_boards.tinyfpga_bx import TinyFPGABXPlatform

    clk_freq = 16e6
    if len(sys.argv) < 2 or sys.argv[1] == "sim":
        m = Module()

        m.submodules.loopback = loopback = DShot(int(clk_freq), 600000, 4)

        sim = sim.Simulator(m)
        sim.add_clock(1.0 / clk_freq)

        sim.add_sync_process(_test_loopback(loopback))

        with sim.write_vcd("dshot.vcd", "dshot.gtkw", traces=loopback.ports()):
            sim.run()
            # while sim.advance():
            #     input("ENTER to continue...")
    elif sys.argv[1] == "build":
        class Board(TinyFPGABXPlatform):
            p = "p1"

        class DshotTest(Elaboratable):
            def __init__(self):
                pass

            def elaborate(self, platform: Platform) -> Module:
                m = Module()

                clk_freq = int(platform.default_clk_frequency)
                m.submodules.dshot = dshot = DShot(clk_freq, 600000, 4)

                led = platform.request("led").o
                m.d.comb += [
                    led.eq(dshot.signals[0])
                ]

                return m

            def ports(self) -> List[Signal]:
                return []

        Board().build(DshotTest())
