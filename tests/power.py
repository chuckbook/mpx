import pyb
import stm

# wut is wakeup counter start value, wucksel is clock source
# counter is decremented at wucksel rate, and wakes the MCU when it gets to 0
# wucksel=0b000 is RTC/16 (RTC runs at 32768Hz)
# wucksel=0b001 is RTC/8
# wucksel=0b010 is RTC/4
# wucksel=0b011 is RTC/2
# wucksel=0b100 is 1Hz clock
# wucksel=0b110 is 1Hz clock with 0x10000 added to wut
# so a 1 second wakeup could be wut=2047, wucksel=0b000, or wut=4095, wucksel=0b001, etc
def wakeup_config(wut, wucksel):
    # disable register write protection
    stm.mem8[stm.RTC + stm.RTC_WPR] = 0xca
    stm.mem8[stm.RTC + stm.RTC_WPR] = 0x53

    # clear WUTE
    stm.mem32[stm.RTC + stm.RTC_CR] &= ~(1 << 10)

    # wait untli WUTWF is set
    while not stm.mem32[stm.RTC + stm.RTC_ISR] & (1 << 2):
        pass

    # program WUT
    stm.mem16[stm.RTC + stm.RTC_WUTR] = wut

    # program WUCKSEL
    stm.mem32[stm.RTC + stm.RTC_CR] |= wucksel & 7

    # set WUTE
    stm.mem32[stm.RTC + stm.RTC_CR] |= 1 << 10

    # set WUTIE to enable interrupts
    stm.mem32[stm.RTC + stm.RTC_CR] |= 1 << 14

    # enable register write protection
    stm.mem8[stm.RTC + stm.RTC_WPR] = 0xff

    # enable external interrupts on line 22
    stm.mem32[stm.EXTI + stm.EXTI_IMR] |= 1 << 22
    stm.mem32[stm.EXTI + stm.EXTI_RTSR] |= 1 << 22

    # clear interrupt flags
    stm.mem32[stm.RTC + stm.RTC_ISR] &= ~(1 << 10)
    stm.mem32[stm.EXTI + stm.EXTI_PR] = 1 << 22

def do_stop():
    # stop the MCU
    pyb.stop()
    # woken; clear interrupt flags
    stm.mem32[stm.RTC + stm.RTC_ISR] &= ~(1 << 10)
    stm.mem32[stm.EXTI + stm.EXTI_PR] = 1 << 22

# demo to wakeup every 2 seconds and toggle led
#wakeup_config(32767, 0b011)
# demo to wakeup every 2 seconds and toggle led
#wakeup_config(10000, 0b000)
# demo to wakeup every 10 seconds and toggle led
wakeup_config( 3599, 0b100)
while True:
    do_stop()
    pyb.LED(2).toggle()
