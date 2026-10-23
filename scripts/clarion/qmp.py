#!/usr/bin/env python3
"""Drive a running Clarion head unit over its QMP socket, without a window.

Start the emulator with a QMP socket (keep the path short, at most 104 bytes):

    qemu-clarion --headless NOR CARD -qmp unix:/tmp/hu.sock,server=on,wait=off

Then, from another terminal:

    qmp.py /tmp/hu.sock wait                 # wait until QMP answers
    qmp.py /tmp/hu.sock shot screen.png      # save the display as PNG
    qmp.py /tmp/hu.sock tap 270 445          # touch at display pixel X,Y
    qmp.py /tmp/hu.sock hmp "info registers" # human monitor command
    qmp.py /tmp/hu.sock set reverse true     # machine property (qom-set)
    qmp.py /tmp/hu.sock raw '{"execute": "query-status"}'
    qmp.py /tmp/hu.sock quit

Tap coordinates are visible display pixels (800x480), origin top left.
The exit status is 0 when QEMU accepted the command; the QMP reply is
printed on stdout.
"""
import argparse
import json
import socket
import sys
import time

WIDTH, HEIGHT = 800, 480
# QY8 binds the touch controller to its display; QY7 takes unbound input.
TOUCH_DEVICES = ("qy8-du", None)


class Qmp:
    def __init__(self, path, timeout=10.0):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.settimeout(timeout)
        self.sock.connect(path)
        self.file = self.sock.makefile("rw")
        self.file.readline()  # greeting
        self.cmd("qmp_capabilities")

    def cmd(self, execute, arguments=None):
        request = {"execute": execute}
        if arguments is not None:
            request["arguments"] = arguments
        self.file.write(json.dumps(request) + "\n")
        self.file.flush()
        while True:
            line = self.file.readline()
            if not line:
                raise ConnectionError("QMP closed the connection")
            reply = json.loads(line)
            if "return" in reply or "error" in reply:
                return reply

    def _input(self, device, events):
        arguments = {"events": events}
        if device:
            arguments["device"] = device
        return self.cmd("input-send-event", arguments)

    def tap(self, x, y, hold=0.15):
        pos = [
            {"type": "abs", "data": {"axis": "x",
                                     "value": round(x * 0x7fff / (WIDTH - 1))}},
            {"type": "abs", "data": {"axis": "y",
                                     "value": round(y * 0x7fff / (HEIGHT - 1))}},
        ]
        for device in TOUCH_DEVICES:
            down = self._input(device, pos + [
                {"type": "btn", "data": {"button": "left", "down": True}}])
            if "error" not in down:
                break
        else:
            return down
        time.sleep(hold)
        return self._input(device, [
            {"type": "btn", "data": {"button": "left", "down": False}}])


def wait_socket(path, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return Qmp(path)
        except (FileNotFoundError, ConnectionRefusedError):
            time.sleep(0.1)
    raise SystemExit(f"no QMP socket after {timeout} s: {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("socket")
    sub = ap.add_subparsers(dest="action", required=True)
    w = sub.add_parser("wait", help="wait for the socket and QMP")
    w.add_argument("--timeout", type=float, default=30.0)
    t = sub.add_parser("tap", help="touch at display pixel X,Y")
    t.add_argument("x", type=int)
    t.add_argument("y", type=int)
    t.add_argument("--hold", type=float, default=0.15,
                   help="seconds between press and release")
    s = sub.add_parser("shot", help="save the display as PNG")
    s.add_argument("file")
    h = sub.add_parser("hmp", help="human monitor command")
    h.add_argument("command")
    p = sub.add_parser("set", help="machine property (qom-set /machine)")
    p.add_argument("property")
    p.add_argument("value")
    r = sub.add_parser("raw", help="any QMP command as JSON")
    r.add_argument("json")
    sub.add_parser("status", help="query-status")
    sub.add_parser("quit", help="stop QEMU")
    a = ap.parse_args()

    if a.action == "wait":
        wait_socket(a.socket, a.timeout)
        print("ready")
        return 0
    q = Qmp(a.socket)
    if a.action == "tap":
        if not (0 <= a.x < WIDTH and 0 <= a.y < HEIGHT):
            ap.error(f"tap is outside the {WIDTH}x{HEIGHT} display")
        reply = q.tap(a.x, a.y, a.hold)
    elif a.action == "shot":
        reply = q.cmd("screendump", {"filename": a.file, "format": "png"})
    elif a.action == "hmp":
        reply = q.cmd("human-monitor-command", {"command-line": a.command})
        if "return" in reply:
            sys.stdout.write(reply["return"])
            return 0
    elif a.action == "set":
        try:
            value = json.loads(a.value)
        except ValueError:
            value = a.value
        reply = q.cmd("qom-set", {"path": "/machine", "property": a.property,
                                  "value": value})
    elif a.action == "raw":
        request = json.loads(a.json)
        reply = q.cmd(request["execute"], request.get("arguments"))
    elif a.action == "status":
        reply = q.cmd("query-status")
    else:
        reply = q.cmd("quit")
    print(json.dumps(reply))
    return 1 if "error" in reply else 0


if __name__ == "__main__":
    sys.exit(main())
