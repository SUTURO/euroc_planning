#!/usr/bin/env python
import sys
import errno
import traceback


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

parent_dead = False
return_values = {'GOT_INPUT': 0, 'NO_INPUT': 1, 'EMPTY_INPUT': 2, 'EXCEPTION': 3}
EOF = False


def write(f, s):
    try:
        f.write(s)
        # f.flush()
        # os.fsync(h.fileno())
    except Exception, e:
        print('[LOGGER] Error writing to file "' + str(s) + '"')
        print('[LOGGER] ' + str(e))


def check_for_input():
    global node
    global logging
    try:
        line = sys.stdin.readline()
        if line == '':
            print('[LOGGER] ' + node + ': EOF.')
            write(h, 'THIS LOGGER TERMINATED CORRECTLY.')
            try:
                h.close()
            except:
                print('[LOGGER] Error closing file ' + str(h))
            return False
        else:
            if logging in [0, 2]:
                write(h, line)
            if not dont_print:
                sys.stdout.write(stdout_prefix + line)
                # sys.stdout.flush()
    except KeyboardInterrupt:
        print('QLQLQL Interrupt.')
    except IOError, e:
        if e.errno != errno.EINTR:
            raise
        else:
            print('[LOGGER] Caught IOError while reading stdin:')
            print('[LOGGER] ' + str(e))
    except:
        print('[LOGGER] Uncaught exception while reading stdin.')
        print(traceback.print_exc())
    return True

r = check_for_input()
while r:
    r = check_for_input()

print('[LOGGER] ' + f + ': Exiting.')