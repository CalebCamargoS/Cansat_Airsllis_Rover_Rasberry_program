#!/usr/bin/env python3
import serial
import time
import binascii

PORT = "/dev/serial0"
BAUD = 115200
P2P = "915000000:7:0:0:16:20"  # Debe ser igual que en el emisor

def at(ser, cmd, wait=0.25, show=True):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    time.sleep(wait)
    out = ser.read(ser.in_waiting or 1).decode(errors="ignore")
    time.sleep(0.08)
    out += ser.read(ser.in_waiting or 1).decode(errors="ignore")
    if show:
        print(f"> {cmd}\n{out.strip()}\n")
    return out

def rearm_rx(ser):
    at(ser, "AT+PRECV=0", show=False, wait=0.05)
    at(ser, "AT+PRECV=65535", show=False, wait=0.05)

if __name__ == "__main__":
    with serial.Serial(PORT, BAUD, timeout=0.15) as s:
        # Asegura que NO está en RX antes de configurar
        at(s, "AT+PRECV=0", show=False)
        at(s, "AT+NWM=0")
        at(s, f"AT+P2P={P2P}")

        # Entrar a escucha continua
        rearm_rx(s)
        print(">> Escuchando paquetes P2P ...\n")

        buf = ""
        while True:
            chunk = s.read(256).decode(errors="ignore")
            if not chunk:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                print(line)
                if line.startswith("+EVT:RXP2P:"):
                    # +EVT:RXP2P:<RSSI>:<SNR>:<HEX>
                    parts = line.split(":")
                    hexpl = parts[-1] if len(parts) >= 4 else ""
                    if hexpl:
                        try:
                            txt = binascii.unhexlify(hexpl).decode("utf-8", errors="ignore")
                            if txt:
                                print("<< Texto:", txt)
                        except Exception:
                            pass
                    # re-armar recepción para el siguiente paquete
                    rearm_rx(s)
                elif "AT_BUSY_ERROR" in line or "P2P_RX_ON" in line:
                    rearm_rx(s)