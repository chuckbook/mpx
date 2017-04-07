import pyb
import stm

def initTIM2Gated():
    stm.mem32[stm.TIM2 + stm.TIM_SMCR] = 0x0000
    stm.mem32[stm.TIM2 + stm.TIM_SMCR] = 0x0070
    stm.mem32[stm.TIM2 + stm.TIM_SMCR] = 0x0075

def initTIM5Gated():
    stm.mem32[stm.TIM5 + stm.TIM_SMCR]  = 0x0000
    stm.mem32[stm.TIM5 + stm.TIM_SMCR] |= 0x0055

def periodTIM2():
    stm.mem16[stm.TIM2 + stm.TIM_ARR]  = 0xffff
    stm.mem16[stm.TIM2 + stm.TIM_ARR+2]  = 0xffff

def periodTIM5():
    stm.mem16[stm.TIM5 + stm.TIM_ARR]  = 0xffff
    stm.mem16[stm.TIM5 + stm.TIM_ARR+2]  = 0xffff

initTIM2Gated()
initTIM5Gated()

timer2 = pyb.Timer(2, prescaler=0, period=0x7FFFFFFF)
pin1 = pyb.Pin(pyb.Pin.board.X6, mode=pyb.Pin.AF_OD, af=pyb.Pin.AF1_TIM2)
timer2.counter(0)

timer5 = pyb.Timer(5, prescaler=0, period=0x7FFFFFFF)
pin5 = pyb.Pin(pyb.Pin.board.X1, mode=pyb.Pin.AF_OD, af=pyb.Pin.AF2_TIM5)
timer5.counter(0)

periodTIM2()
periodTIM5()

def c():
    global timer2, timer5
    timer2.counter(0)
    timer5.counter(0)

def r():
    while(1):
        pyb.delay(50)
        print("TIM2")
        print(timer2.counter())
        print(pin1.value())
        print("TIM5")
        print(timer5.counter())
        print(pin5.value())
        print("\n")

r()
