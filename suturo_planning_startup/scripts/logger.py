#!/usr/bin/env python
import time
import sys
import os
import signal
import atexit
import select
import errno

__loop = True


def exit_handler(signum=None, frame=None):
    if signum is not None:
        global __loop
        global parent_dead
        print('[LOGGER] #################################################################')
        print('[LOGGER] Exit handler for logger ' + node + ' on signal ' + str(signum) + '.')
        print('[LOGGER] Setting __loop to False.')
        __loop = False
        print('[LOGGER] Emptying input buffer.')
        no_input_counter = 0
        while no_input_counter < 13337:
            r, l = check_for_input()
            # print('[LOGGER] ' + node + ': parent_dead: ' + str(parent_dead))
            # print('[LOGGER] ' + node + ': ret: ' + str(r))
            # print('[LOGGER] ' + node + ': line: "' + str(l) + '"')
            if parent_dead and r == return_values['NO_INPUT']:
                break
            elif parent_dead and r == return_values['EMPTY_INPUT']:
                no_input_counter += 1
            else:
                no_input_counter = 0
        print('[LOGGER] Closing file.')
        try:
            h.write('THIS LOGGER TERMINATED CORRECTLY.')
            h.flush()
            os.fsync(h.fileno())
            h.close()
        except:
            print('[LOGGER] Error closing file ' + str(h))
        print('[LOGGER] Exiting exit_handler.')
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

parent_pid = int(sys.argv[6])
h = open(f, 'w')
signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)
atexit.register(exit_handler)

parent_dead = False
return_values = {'GOT_INPUT': 0, 'NO_INPUT': 1, 'EMPTY_INPUT': 2, 'EXCEPTION': 3}


def is_alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True


def check_for_input():
    global node
    global logging
    global parent_pid
    global parent_dead
    try:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                line = sys.stdin.readline().rstrip()
                try:
                    if line == '':
                        return return_values['EMPTY_INPUT'], line
                    else:
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
                return return_values['GOT_INPUT'], line
            except IOError, e:
                if e.errno != errno.EINTR:
                    raise
                else:
                    print('[LOGGER] Uncaught exception while reading stdin.')
                    print('[LOGGER] Caught exception while reading stdin:')
                    print('[LOGGER] ' + str(e))
        else:
            return return_values['NO_INPUT'], None
    except select.error, e:
        if e[0] != errno.EINTR:
            print('[LOGGER] Uncaught exception while doing select on stdin.')
            raise
        else:
            print('[LOGGER] Caught exception while doing select on stdin:')
            print('[LOGGER] ' + str(e))
    finally:
        if not parent_dead:
            if is_alive(parent_pid):
                parent_dead = True
            else:
                print('[LOGGER] ' + node + ': Parent process with pid ' + str(parent_pid) + ' died.')

while __loop:
    ret, line = check_for_input()
    if ret != return_values['GOT_INPUT']:
        time.sleep(0.05)

print('[LOGGER] ' + node + ': Exiting.')