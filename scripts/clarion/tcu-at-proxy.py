#!/usr/bin/env python3
"""Minimal Ficosa AT proxy for the QY8 CDC ACM TCU model."""

import argparse
import json
import socket
import time


def qmp_set_serial_state(path):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as qmp:
        qmp.connect(path)
        stream = qmp.makefile("rwb", buffering=0)
        stream.readline()  # greeting
        stream.write(b'{"execute":"qmp_capabilities"}\r\n')
        stream.readline()
        command = {
            "execute": "qom-set",
            "arguments": {
                "path": "/machine/peripheral/qy8-tcu",
                "property": "serial-state",
                "value": 3,
            },
        }
        stream.write((json.dumps(command) + "\r\n").encode())
        reply = json.loads(stream.readline())
        if "error" in reply:
            raise RuntimeError(reply["error"])


def response(command):
    upper = command.upper()
    if upper in ("ATE0", "ATE0&C1&D2", "ATX4"):
        return b"\r\nOK\r\n", False
    if upper == "AT+CMER?" or upper.startswith("AT+CMER="):
        return b"\r\nOK\r\n", False
    if upper == "AT+CNUM":
        return b"\r\nOK\r\n", False
    if upper == "ATS7=60":
        return b"\r\nOK\r\n", False
    if upper == "ATI":
        return b"\r\nTCU Gen2 Kai CX.6_RC01\r\nOK\r\n", False
    if upper == "AT+CGDCONT=?":
        return b'\r\n+CGDCONT: (1-3),("IP","IPV6","IPV4V6")\r\nOK\r\n', False
    if upper == "AT+CGDCONT?":
        return b'\r\n+CGDCONT: 1,"IP","internet","0.0.0.0",0,0\r\nOK\r\n', False
    if upper.startswith("AT+CGDCONT=") or upper == "AT":
        return b"\r\nOK\r\n", False
    if upper.startswith("ATD*99***") and upper.endswith("#"):
        return b"\r\nCONNECT\r\n", True
    return b"\r\nERROR\r\n", False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, help="QEMU chardev Unix socket")
    parser.add_argument("--transcript", required=True, help="JSONL transcript path")
    parser.add_argument("--serial-state-qmp", help="QMP socket for B2 DCD/DSR")
    args = parser.parse_args()

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn, \
            open(args.transcript, "a", encoding="utf-8") as log:
        while True:
            try:
                conn.connect(args.socket)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.1)
        pending = bytearray()
        connected = False
        while True:
            data = conn.recv(4096)
            if not data:
                break
            if connected:
                log.write(json.dumps({"event": "post_connect_data",
                                      "hex": data.hex()}, sort_keys=True) + "\n")
                log.flush()
                continue
            pending.extend(data)
            while b"\r" in pending:
                raw, _, rest = pending.partition(b"\r")
                pending = bytearray(rest)
                command = raw.decode("ascii", errors="replace").strip("\n")
                reply, is_connect = response(command)
                log.write(json.dumps({"event": "command", "command": command,
                                      "reply": reply.decode("ascii").replace("\r", "\\r").replace("\n", "\\n"),
                                      "reply_hex": reply.hex()}, sort_keys=True) + "\n")
                log.flush()
                conn.sendall(reply)
                if is_connect:
                    connected = True
                    if args.serial_state_qmp:
                        qmp_set_serial_state(args.serial_state_qmp)
                        log.write(json.dumps({"event": "serial_state", "dcd": 1,
                                              "dsr": 1}) + "\n")
                        log.flush()
                    break


if __name__ == "__main__":
    main()
