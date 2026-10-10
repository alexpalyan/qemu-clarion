#!/usr/bin/env python3
# tap.py SOCK X Y  — press and release at screen pixel X,Y (800x480) via QMP input events
import json, socket, sys, time
s = socket.socket(socket.AF_UNIX); s.connect(sys.argv[1]); f = s.makefile('rw')
def cmd(c, **a):
    f.write(json.dumps({'execute': c, 'arguments': a}) + '\n'); f.flush()
    while True:
        r = json.loads(f.readline())
        if 'return' in r or 'error' in r: return r
json.loads(f.readline()); cmd('qmp_capabilities')
x = int(int(sys.argv[2]) * 32767 / 799); y = int(int(sys.argv[3]) * 32767 / 479)
pos = [{'type': 'abs', 'data': {'axis': 'x', 'value': x}}, {'type': 'abs', 'data': {'axis': 'y', 'value': y}}]
# the touch model listens on the display's console only
print(cmd('input-send-event', device='qy8-du', events=pos + [{'type': 'btn', 'data': {'down': True, 'button': 'left'}}]))
time.sleep(0.3)
print(cmd('input-send-event', device='qy8-du', events=pos + [{'type': 'btn', 'data': {'down': False, 'button': 'left'}}]))
