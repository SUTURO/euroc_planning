#!/usr/bin/env python
import time
import sys
import os
import signal
import atexit
import select
import errno

__loop = True
__no_input_counter = 0


def exit_handler(signum=None, frame=None):
    if signum is not None:
        global __loop
        global __no_input_counter
        print('[LOGGER] #################################################################')
        print('[LOGGER] Exit handler for logger ' + f + ' on signal ' + str(signum) + '.')
        if __no_input_counter < 1000:
            print('[LOGGER] Emptying input buffer.')
        while __no_input_counter < 1000:
            check_for_input()
        print('[LOGGER] Setting __loop to False.')
        __loop = False
        print('[LOGGER] Closing file.')
        try:
            h.close()
        except:
            print('[LOGGER] Error closing file ' + str(h))
        print('[LOGGER] Exiting.')
        print('[LOGGER] #################################################################')


f = sys.argv[1]
node = sys.argv[2]
logging = int(sys.argv[3])
if sys.argv[4] == 'True':
    dont_print = True
else:
    dont_print = False
if sys.argv[5] == 'True':
    stdout_prefix = '[' + node + '] '
else:
    stdout_prefix = ''
h = open(f, 'w')
signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)
atexit.register(exit_handler)

def check_for_input():
    global __no_input_counter
    global node
    global logging
    try:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                line = sys.stdin.readline().rstrip()
                try:
                    if line == '':
                        __no_input_counter += 1
                    else:
                        __no_input_counter = 0
                        if logging in [0, 2]:
                            h.write(line + '\n')
                            h.flush()
                            os.fsync(h.fileno())
                except Exception, e:
                    print('[LOGGER] Error writing to file ' + str(h))
                    print('[LOGGER] ' + str(e))
                if line != '' and not dont_print:
                    print(stdout_prefix + line)
                    sys.stdout.flush()
            except IOError, e:
                if e.errno != errno.EINTR:
                    raise
                else:
                    print('[LOGGER] Uncaught exception while reading stdin.')
                    print('[LOGGER] Caught exception while reading stdin:')
                    print('[LOGGER] ' + str(e))
        else:
            __no_input_counter += 1
            time.sleep(0.05)
    except select.error, e:
        if e[0] != errno.EINTR:
            print('[LOGGER] Uncaught exception while doing select on stdin.')
            raise
        else:
            print('[LOGGER] Caught exception while doing select on stdin:')
            print('[LOGGER] ' + str(e))

while __loop:
    check_for_input()