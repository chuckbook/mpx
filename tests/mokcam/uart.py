from pyb import UART

# test we can correctly create by id or name
for bus in (-1, 0, 1, 2, 3, 4, 5, 6, 7, "XA", "XB", "YA", "YB", "Z"):
    try:
        UART(bus, 9600)
        print("UART", bus)
    except ValueError:
        print("ValueError", bus)

uart = UART(2)
uart = UART(2, 9600)
uart = UART(2, 9600, bits=8, parity=None, stop=1)
print(uart)

uart.init(2400)
print(uart)

print(uart.any())
print(uart.write('123'))
print(uart.write(b'abcd'))
print(uart.writechar(1))

# make sure this method exists
uart.sendbreak()

# non-blocking mode
uart = UART(2, 9600, timeout=0)
print(uart.write(b'1'))
print(uart.write(b'abcd'))
print(uart.writechar(1))
print(uart.read(100))
