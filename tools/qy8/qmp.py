#!/usr/bin/env python3
# qmp.py SOCK 'hmp command' ['hmp command' ...]
import json, socket, sys
s = socket.socket(socket.AF_UNIX); s.connect(sys.argv[1]); f = s.makefile('rw')
def cmd(c, **a):
    f.write(json.dumps({'execute': c, 'arguments': a}) + '\n'); f.flush()
    while True:
        r = json.loads(f.readline())
        if 'return' in r or 'error' in r: return r
json.loads(f.readline()); cmd('qmp_capabilities')
for h in sys.argv[2:]:
    r = cmd('human-monitor-command', **{'command-line': h})
    print(r.get('return', r.get('error')), end='')
