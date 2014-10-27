from mercurial.match import exact
from os import kill

__author__ = 'moritz'

from subprocess import check_output, CalledProcessError


def exterminate(killme, signal):
    if killme == '':
        return
    pids=[]
    try:
        pids = check_output(["pgrep", "-P", str(killme)]).split('\n')
    except:
        pids=[]
    for pid in pids:
        exterminate(pid)
    kill(killme, signal)