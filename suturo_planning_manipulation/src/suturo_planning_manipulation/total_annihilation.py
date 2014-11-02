import os
from sets import Set
import signal
from subprocess import check_output

__author__ = 'moritz bleibuntreu'


def exterminate(killme, signal, r=False):
    #print('Executing exterminate')
    print('exterminate. Kill pid: ' + str(killme) + ' with signal ' + str(signal))
    if r:
        pids = Set()
        getpids(pids, killme)
        for pid in pids:
            try:
                #print('Killing process ' + str(pid))
                os.kill(int(pid), signal)
            except OSError, e:
                #print "ASDException: fgsfds " + e.strerror
                pass
    else:
        if killme == '':
            print('Nix zu killen!')
            return
        else:
            try:
                os.killpg(os.getpgid(killme), signal)
            except Exception, e:
                print('FDO on killpg: ' + str(e))
                print(e)
            try:
                os.kill(killme, signal)
            except Exception, e:
                print('FDO on kill: ' + str(e))
                print(e)


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