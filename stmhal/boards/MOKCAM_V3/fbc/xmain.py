import pyb
import sys
import micropython
stupt=pyb.micros()
micropython.alloc_emergency_exception_buf(200)

led=pyb.LED(1)
led.on()
import stm

def hexd(a,n=4):
 sbuf=''
 if type(a) == type(int()):
  if (a < 0):
   a &= 0x7fffffff
  for i in range(0,n,4):
   if i & 0xf == 0:
    if len(sbuf): print(sbuf)
    sbuf='%04x%04x ' % (((a+i)>>16)&0xffff,(a+i)&0xffff)
   ba=stm.mem32[a+i].to_bytes(4, 'little')
   #sbuf+=' %04x%04x' % (stm.mem16[(a+i+2)], stm.mem16[(a+i)])
   sbuf+=' %02x%02x%02x%02x' % (ba[3],ba[2],ba[1],ba[0])
 elif type(a) in [type(bytearray()),type(bytes())]:
  for i in range(0,len(a)&~3,4):
   if i & 0xf == 0:
    if len(sbuf): print(sbuf)
    sbuf='%04x ' % (i)
   sbuf+=' %02x%02x%02x%02x' % (a[i+3],a[i+2],a[i+1],a[i+0])
 elif type(a) in [type(str())]:
  for i in range(0,len(a)&~3,4):
   if i & 0xf == 0:
    if len(sbuf): print(sbuf)
    sbuf='%04x ' % (i)
   sbuf+=' %02x%02x%02x%02x' % (ord(a[i+3]),ord(a[i+2]),ord(a[i+1]),ord(a[i+0]))
 print(sbuf)

rtc=pyb.RTC()
info0=rtc.info()
ts0=pyb.micros()
res=rtc.datetime()
stm.mem32[stm.RTC+0x60]+=1
if False:
#if res[6] // 10 != 0:
 stm.mem32[stm.RTC+stm.RTC_BKP17R]=stm.mem32[stm.RTC+stm.RTC_BKP16R]
 stm.mem32[stm.RTC+stm.RTC_BKP16R]=(stm.mem32[stm.RTC] << 8) | (stm.mem32[stm.RTC+stm.RTC_SSR] & 0xff)
 stm.mem32[stm.RTC+stm.RTC_BKP19R]+=1
 import mymain

dt=rtc.datetime()
info1=rtc.info()
ts1=pyb.micros()

print('\n%d us 0x%x %d us 0x%x %d us LSx_dt: %d us' % (stupt, info0, ts0-stupt, info1, ts1-stupt, ts1-ts0))
print('%d-%02d-%02d %02d:%02d:%02d.%06d' % (res[0], res[1], res[2], res[4], res[5], res[6], res[7]))
try:
    adc=pyb.ADCAll(12,0)
except:
    adc=pyb.ADCAll(12)

try:
    vref = adc.read_vref()
except:
    vref = 3.3

print('Tcore: %.1f C Vcore_ref: %.3f V Vbat: %.3f V Vref: %.3f V' % (adc.read_core_temp(), adc.read_core_vref(), adc.read_core_vbat(), vref))
print('freq: ', pyb.freq())
print('LTE:')
hexd(stm.RCC+0x70, 0x10)
led.off()
