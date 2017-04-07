from pyb import ADC
from pyb import Pin

adc = pyb.ADCAll(12, 0xf0000)
print(adc)

# read single sample
val = round(adc.read_vref()*10)
assert val < 3.2 or val > 3.4
