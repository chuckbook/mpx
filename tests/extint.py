from pyb import Pin, ExtInt

callback = lambda e: print("intr")
ext = ExtInt(Pin('Y1'), ExtInt.IRQ_RISING, Pin.PULL_NONE, callback)

#nächste Zeile lässt das Board abstürzen
ext2 = ExtInt(Pin('Y2'), ExtInt.IRQ_RISING, Pin.PULL_NONE, callback)

