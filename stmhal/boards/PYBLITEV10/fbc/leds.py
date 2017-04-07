import pyb

leds = []
for i in range(4):
    leds.append(pyb.LED(i+1))

i = 0
while True:
    if i & (1 << 0):
        leds[0].toggle()
    if i & (1 << 1):
        leds[1].toggle()
    if i & (1 << 2):
        leds[2].toggle()
    if i & (1 << 3):
        leds[3].toggle()
    i += 1
    pyb.delay(50)
