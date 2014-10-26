#!/usr/bin/env python
import sys
import atexit
import os

__loop = True


def exit_handler():
    print('Exit handler for logger ' + f)
    global __loop
    try:
        __loop = False
        h.close()
    except:
        print('Error closing file ' + str(h))

f = sys.argv[1]
h = open(f, 'w')
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


