from os import kill
from sets import Set
import signal
from subprocess import check_output

__author__ = 'moritz bleibuntreu'



def exterminate(killme, signal):
    print('Executing exterminate')
    print('killme: ' + str(killme))
    if killme == '':
        return
    pids = Set()
    killedpids = Set()
    getpids(pids, killme)
    for pid in pids:
        try:
            print('Killing process ' + str(pid))
            kill(int(pid), signal)
        except OSError, e:
            print "ASDException: fgsfds " + e.strerror

def getpids(pidset, pid):
    pids = []
    try:
        pids = check_output(["pgrep", "-P", str(pid)]).split('\n')
    except:
        pids = []
    for childpid in pids:
        if childpid == '':
            continue
        getpids(pidset, childpid)
    pidset.add(pid)