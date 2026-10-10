#!/usr/bin/env python3
# shell.py SOCK BOOT_SECS cmd1 cmd2 ...  — talk to the guest debug console
import socket, sys, time, select
s = socket.socket(socket.AF_UNIX)
for _ in range(50):
    try: s.connect(sys.argv[1]); break
    except OSError: time.sleep(0.2)
def drain(secs):
    out = b''; end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select([s], [], [], 0.2)
        if r:
            d = s.recv(65536)
            if not d: break
            out += d
    return out.decode('latin1')
boot = drain(float(sys.argv[2]))
print('=== boot output (last 1500 chars) ===\n' + boot[-1500:])
for c in sys.argv[3:]:
    s.sendall(c.encode() + b'\r')
    print(f'=== >> {c!r} ===\n' + drain(4))
