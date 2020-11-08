from sys import platform
from typing import List
from nmigen import Elaboratable, Signal, Module, Cat
from nmigen.build import *
from nmigen import sim
from abc import ABCMeta, abstractmethod


class UARTBase(Elaboratable, metaclass=ABCMeta):
    def __init__(self, clk_freq, baud):
        self.data = Signal(8)
        self.signal = Signal()
        self.ready = Signal()
        self.accepted = Signal()
        self.bits = Signal(range(8), reset=7)
        self.clk_per_bit = int(clk_freq // baud)
        self.count = Signal(range(self.clk_per_bit), reset=self.clk_per_bit)
        self.fsm = None

    def ports(self) -> List[Signal]:
        return [self.data,
                self.signal,
                self.ready,
                self.accepted,
                self.bits,
                self.clk_per_bit,
                self.count
                ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        with m.FSM(reset="IDLE") as self.fsm:
            with m.State("IDLE"):
                self.Idle(m)

            with m.State("START"):
                self.Start(m)

            with m.State("DATA"):
                self.Data(m)

            with m.State("STOP"):
                self.Stop(m)

            with m.State("DONE"):
                self.Done(m)

            # with m.State("ERROR"):
            #     self.Error(m)

        return m

    @abstractmethod
    def Idle(self, m: Module):
        pass

    @abstractmethod
    def Start(self, m: Module):
        pass

    @abstractmethod
    def Data(self, m: Module):
        pass

    @abstractmethod
    def Stop(self, m: Module):
        pass

    def Done(self, m: Module):
        with m.If(self.accepted):
            m.next = "IDLE"

    # @abstractmethod
    # def Error(self, m: Module):
    #     pass


# -----------------------------------------------------------------------------
# UART Receiver
# -----------------------------------------------------------------------------

class UartRx(UARTBase):
    def __init__(self, clk_freq, baud):
        UARTBase.__init__(self, clk_freq, baud)

    # * Wait for signal to go low
    def Idle(self, m: Module):
        m.d.sync += self.ready.eq(0)
        with m.If(self.signal == 0):
            m.d.sync += self.count.eq(self.count.reset)
            m.next = "START"

    # * Receive start bit
    def Start(self, m: Module):
        # Count down bit timer
        m.d.sync += self.count.eq(self.count - 1)

        # Sample signal at middle of bit period
        with m.If(self.count == self.clk_per_bit // 2):
            # Start bit not kept low
            with m.If(self.signal == 1):
                m.next = "IDLE"  # "ERROR"
        # Bit period is complete
        with m.Elif(self.count == 0):
            m.next = "DATA"
            m.d.sync += self.bits.eq(self.bits.reset)

    # * Receive all data bits
    def Data(self, m: Module):
        # Count down bit timer
        m.d.sync += self.count.eq(self.count - 1)

        # Sample signal at middle of bit period
        with m.If(self.count == int(self.clk_per_bit // 2)):
            # Set bit
            m.d.sync += self.data.bit_select(self.bits.reset -
                                             self.bits, 1).eq(self.signal)

        # Done with current bit?
        with m.Elif(self.count == 0):
            m.d.sync += [
                self.bits.eq(self.bits - 1),
                self.count.eq(self.count.reset)
            ]
            # Done with all bits?
            with m.If(self.bits == 0):
                m.next = "STOP"

    def Stop(self, m: Module):
        # Count down bit timer
        m.d.sync += self.count.eq(self.count - 1)

        # Sample signal at middle of bit period
        with m.If(self.count == self.clk_per_bit // 2):
            # Start bit not kept low
            with m.If(self.signal == 1):
                m.next = "IDLE"  # "ERROR"
        # Bit period is complete
        with m.Elif(self.count == 0):
            m.next = "DONE"
            m.d.sync += [
                self.bits.eq(self.bits.reset),
                self.ready.eq(1)
            ]


# -----------------------------------------------------------------------------
# UART Transmitter
# -----------------------------------------------------------------------------

class UartTx(UARTBase):
    def __init__(self, clk_freq, baud):
        UARTBase.__init__(self, clk_freq, baud)
        self.send = Signal()

    def elaborate(self, platform: Platform) -> Module:
        m = super().elaborate(platform)
        m.d.comb += self.send.eq(self.ready)
        return m

    # * Wait for signal to go low
    def Idle(self, m: Module):
        m.d.sync += [
            self.accepted.eq(0),
            self.signal.eq(1)
        ]

        with m.If(self.ready == 1):
            m.d.sync += self.count.eq(self.count.reset)
            m.next = "START"

    # * Receive start bit
    def Start(self, m: Module):
        # Count down bit timer
        m.d.sync += [
            self.count.eq(self.count - 1),
            self.signal.eq(0)
        ]

        # Bit period is complete
        with m.If(self.count == 0):
            m.next = "DATA"
            m.d.sync += self.bits.eq(self.bits.reset)

    # * Receive all data bits
    def Data(self, m: Module):
        # Count down bit timer
        m.d.sync += [
            self.count.eq(self.count - 1),
            self.signal.eq(self.data.bit_select(
                self.bits.reset - self.bits, 1))
        ]

        # Done with current bit?
        with m.If(self.count == 0):
            m.d.sync += [
                self.bits.eq(self.bits - 1),
                self.count.eq(self.count.reset)
            ]
            # Done with all bits?
            with m.If(self.bits == 0):
                m.next = "STOP"

    def Stop(self, m: Module):
        # Count down bit timer
        m.d.sync += [
            self.count.eq(self.count - 1),
            self.signal.eq(0)
        ]
        # Bit period is complete
        with m.If(self.count == 0):
            m.next = "DONE"
            m.d.sync += [
                self.bits.eq(self.bits.reset),
                self.accepted.eq(1)
            ]


# -----------------------------------------------------------------------------
# UART Transceiver
# -----------------------------------------------------------------------------
class Uart(Elaboratable):
    def __init__(self, clk_freq, baud):
        self.baud = baud
        self.clk_freq = clk_freq
        self.rx = UartRx(
            clk_freq=clk_freq, baud=self.baud)
        self.tx = UartTx(
            clk_freq=clk_freq, baud=self.baud)

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.submodules.rx = self.rx
        m.submodules.tx = self.tx

        return m


def _test_loopback(dut):
    data = [0b10101010, 0x0F, 0xFF, 0x7F]

    def run_test():
        for d in data:
            print(d)
            yield dut.uart.tx.data.eq(d)
            yield dut.uart.tx.ready.eq(1)

            while(yield dut.uart.tx.accepted == 0):
                yield
            # yield dut.tx.ready.eq(0)
            while(yield dut.uart.rx.ready) == 0:
                yield

            # print(int(dut.rx.data), d)
            # assert(yield dut.rx.data) == d
            yield dut.uart.rx.accepted.eq(1)
            yield
            yield dut.uart.rx.accepted.eq(0)
            yield

    return run_test


class _LoopbackTest(Elaboratable):
    def __init__(self, clk_freq, baud):
        self.uart = None
        self.baud = baud
        self.clk_freq = clk_freq

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.submodules.uart = self.uart = Uart(
            clk_freq=clk_freq, baud=self.baud)

        m.d.comb += [
            self.uart.rx.signal.eq(self.uart.tx.signal),
        ]
        return m


if __name__ == "__main__":
    import sys
    clk_freq = 12e6
    if len(sys.argv) < 2 or sys.argv[1] == "sim":
        m = Module()

        m.submodules.loopback = loopback = _LoopbackTest(int(clk_freq), 115200)

        sim = sim.Simulator(m)
        sim.add_clock(1.0 / clk_freq)

        sim.add_sync_process(_test_loopback(loopback))
        with sim.write_vcd("uart.vcd", "uart.gtkw", traces=[loopback.uart.tx.signal, loopback.uart.rx.signal]):
            sim.run()
