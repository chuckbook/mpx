import utime


ITERS = 20000000

def run(f):
    t = utime.time()
    f(ITERS)
    t = utime.time() - t
    print(t)
