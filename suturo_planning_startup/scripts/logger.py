#!/usr/bin/env python
import sys
import os
import signal
import atexit
import threading

__loop = True
__lock = threading.Lock()


def exit_handler(signum=None, frame=None):
    print('Exit handler for logger ' + f + ' on signal' + str(signum) + '.')
    global __loop
    __lock.acquire()
    __loop = False
    __lock.release()
    try:
        h.close()
    except:
        print('Error closing file ' + str(h))
    sys.exit(0)

f = sys.argv[1]
h = open(f, 'w')
signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)
atexit.register(exit_handler)

__lock.acquire()
while __loop:
    line = sys.stdin.readline().rstrip()
    try:
        if line == '':
            pass
        else:
            h.write(line + '\n')
            h.flush()
            os.fsync(h.fileno())
    except:
        print('Error writing to file ' + str(h))
    finally:
        __lock.release()
    if line != '':
        print(line)
    __lock.acquire()