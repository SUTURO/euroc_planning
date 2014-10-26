#!/usr/bin/env python
import sys
import os
import signal
import atexit

__loop = True


def exit_handler(signum=None, frame=None):
    print('Exit handler for logger ' + f + ' on signal' + str(signum))
    global __loop
    __loop = False
    try:
        h.close()
    except:
        print('Error closing file ' + str(h))

f = sys.argv[1]
h = open(f, 'w')
signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)
atexit.register(exit_handler)

while __loop:
    line = sys.stdin.readline().rstrip()
    if line == '':
        continue
    try:
        h.write(line + '\n')
        h.flush()
        os.fsync(h.fileno())
    except:
        print('Error writing to file ' + str(h))
    print(line)


