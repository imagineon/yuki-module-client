#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python YUKI-Module Client – Debug/Tooling
----------------------------------
- UART (pyserial) based client for the YUKI-Module TLV protocol
- Mandatory CRC-16/CCITT-FALSE over header + payload (big-endian)
- Synchronous requests/responses and asynchronous geolocation reports
- Integrated CLI for quick debug queries

Installation:
  pip install pyserial

Examples:
  python yuki_module_client.py --port /dev/ttyUSB0 status
  python yuki_module_client.py -p /dev/ttyUSB0 version
  python yuki_module_client.py -p /dev/ttyUSB0 imei
  python yuki_module_client.py -p /dev/ttyUSB0 iccid
  python yuki_module_client.py -p /dev/ttyUSB0 pubkey
  python yuki_module_client.py -p /dev/ttyUSB0 set 0x1234 uint8 42
  python yuki_module_client.py -p /dev/ttyUSB0 geo-request
  python yuki_module_client.py -p /dev/ttyUSB0 poll   # waits for incoming frames (incl. GEO_RPT)

Note:
  On Windows the port is something like "COM5"
"""

from __future__ import annotations

import argparse
import logging
import struct
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

try:
    import serial  # pyserial
except ImportError as e:
    print("pyserial not installed. Please install with 'pip install pyserial'.", file=sys.stderr)
    raise

# ---------------------------------------------------------------------
# Protokoll-Konstanten
# ---------------------------------------------------------------------

YUKI_MODULE_MAX_TLV_LENGTH = 511

# Commands (Type)
CMD_GET_PUBKEY = 0x00
CMD_GET_IMEI   = 0x01
CMD_GET_ICCID  = 0x02
CMD_SET        = 0x04
CMD_SYNC       = 0x05
CMD_VERSION    = 0x06
CMD_STATUS     = 0x07
CMD_GEO_REQ    = 0x08
CMD_GEO_RPT    = 0x09

# Types
TYPE_INT32   = 0x01
TYPE_INT16   = 0x02
TYPE_INT8    = 0x03
TYPE_UINT32  = 0x04
TYPE_UINT16  = 0x05
TYPE_UINT8   = 0x06
TYPE_BOOL    = 0x07
TYPE_UUID    = 0x08
TYPE_FLOAT   = 0x09
TYPE_DATETIME= 0x0A
TYPE_DOUBLE  = 0x0B
TYPE_BIN     = 0x0C
TYPE_UINT64  = 0x0D
TYPE_STRING  = 0x0E
TYPE_IINT64  = 0x0F

# Error codes
ERR_OK        = 0x00
ERR_CMD       = 0x01
ERR_ARG       = 0x02
ERR_BUSY      = 0x03
ERR_INTERNAL  = 0xFF

ERR_STR = {
    ERR_OK: "OK",
    ERR_CMD: "CMD",
    ERR_ARG: "ARG",
    ERR_BUSY: "BUSY",
    ERR_INTERNAL: "INTERNAL",
}

# ---------------------------------------------------------------------
# Datenstrukturen
# ---------------------------------------------------------------------

@dataclass
class YukiModuleGeo:
    fix_type: int      # 0=none, 1=2D, 2=3D
    sats: int
    ts_utc: int        # UNIX time
    lat_e7: int        # WGS84 * 1e-7
    lon_e7: int
    alt_cm: int        # cm (MSL)
    hdop_centi: int    # HDOP * 100

# ---------------------------------------------------------------------
# CRC-16/CCITT-FALSE
# ---------------------------------------------------------------------

def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no final XOR)
    over all bytes in 'data'.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ---------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------

def pack_header(t: int, l: int) -> bytes:
    """
    Header (2 bytes):
      - t (7 bits) left-aligned in byte 0 (we store t << 1)
      - l (9 bits): bit 8 -> bit 0 of byte 0, bits 7..0 -> byte 1
    """
    if t > 0x7F or l > YUKI_MODULE_MAX_TLV_LENGTH:
        raise ValueError("invalid Header: t or l out of range")
    b0 = ((t & 0x7F) << 1) | ((l >> 8) & 0x01)
    b1 = l & 0xFF
    return bytes([b0, b1])

def unpack_header(h: bytes) -> Tuple[int, int]:
    if len(h) != 2:
        raise ValueError("Header-Length != 2")
    t = h[0] >> 1
    l = ((h[0] & 0x01) << 8) | h[1]
    return t, l

# ---------------------------------------------------------------------
# UART Client
# ---------------------------------------------------------------------

GeoCallback = Callable[[YukiModuleGeo], None]

class YukiModuleClient:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0,
                 geo_callback: Optional[GeoCallback] = None,
                 logger: Optional[logging.Logger] = None) -> None:
        self.log = logger or logging.getLogger("yuki_module")
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,        # read timeout (s)
            write_timeout=timeout,  # write timeout (s)
        )
        self.geo_callback = geo_callback

    # ------------- Low-level IO -------------

    def _read_exact(self, n: int) -> bytes:
        """Reads exactly n bytes or raises Timeout/IOError."""
        data = bytearray()
        while len(data) < n:
            chunk = self.ser.read(n - len(data))
            if not chunk:
                raise TimeoutError("UART read timeout")
            data.extend(chunk)
        return bytes(data)

    def _write_all(self, data: bytes) -> None:
        total = 0
        while total < len(data):
            n = self.ser.write(data[total:])
            if n is None or n == 0:
                raise TimeoutError("UART write timeout")
            total += n

    # ------------- Framing -------------

    def send_frame(self, t: int, payload: bytes) -> None:
        if len(payload) > YUKI_MODULE_MAX_TLV_LENGTH:
            raise ValueError("Payload too long")
        header = pack_header(t, len(payload))
        crc = crc16_ccitt_false(header + payload)
        frame = header + payload + struct.pack(">H", crc)
        if self.log.isEnabledFor(logging.DEBUG):
            self.log.debug("TX: t=0x%02X, l=%d, crc=0x%04X", t, len(payload), crc)
        self._write_all(frame)

    def recv_frame(self) -> Tuple[int, bytes]:
        header = self._read_exact(2)
        t, l = unpack_header(header)
        if l > YUKI_MODULE_MAX_TLV_LENGTH:
            raise IOError("Protocol error: Length > Limit")
        payload = self._read_exact(l) if l else b""
        crc_rx = self._read_exact(2)
        (crc_rx_val,) = struct.unpack(">H", crc_rx)
        crc_calc = crc16_ccitt_false(header + payload)
        if crc_calc != crc_rx_val:
            raise IOError("CRC-Error (calc=0x%04X, rx=0x%04X)" % (crc_calc, crc_rx_val))
        if self.log.isEnabledFor(logging.DEBUG):
            self.log.debug("RX: t=0x%02X, l=%d, crc=0x%04X OK", t, l, crc_rx_val)
        return t, payload

    # ------------- Requests -------------

    def request(self, t: int, payload: bytes = b"") -> Tuple[int, bytes]:
        """Sends a request and waits for a response. Returns (err, data)."""
        self.send_frame(t, payload)
        rt, rp = self.recv_frame()
        if len(rp) == 0:
            raise IOError("Protocol error: empty response")
        err = rp[0]
        return err, rp[1:]

    # ------------- High-level API -------------

    def sync(self) -> int:
        err, _ = self.request(CMD_SYNC)
        return err

    def status(self) -> int:
        err, _ = self.request(CMD_STATUS)
        return err

    def version(self) -> Tuple[int, str]:
        err, data = self.request(CMD_VERSION)
        return err, data.decode('utf-8', errors='replace')

    def get_imei(self) -> Tuple[int, str]:
        err, data = self.request(CMD_GET_IMEI)
        return err, data.decode('utf-8', errors='replace')

    def get_iccid(self) -> Tuple[int, str]:
        err, data = self.request(CMD_GET_ICCID)
        return err, data.decode('utf-8', errors='replace')

    def get_pubkey(self) -> Tuple[int, bytes]:
        err, data = self.request(CMD_GET_PUBKEY)
        if len(data) != 64:
            self.log.warning("PubKey-lenght unexpected: %d", len(data))
        return err, data

    def set_value(self, param_id: int, vtype: int, data: bytes, read_only: bool = False) -> int:
        # Payload according to CMD_SET
        flags = 0x10 if read_only else 0x00
        if vtype in (TYPE_STRING, TYPE_BIN):
            if len(data) > (YUKI_MODULE_MAX_TLV_LENGTH - 6):
                raise ValueError("Data too long")
            payload = struct.pack(">HBBBH", param_id, flags, vtype, 0, 0)  # Platzhalter für Länge?
            # Korrigiert: ID (2), Flags (1), Type (1), Len (2), Data
            payload = struct.pack(">HBBH", param_id, flags, vtype, len(data)) + data
            err, _ = self.request(CMD_SET, payload)
            return err
        else:
            if len(data) > (YUKI_MODULE_MAX_TLV_LENGTH - 4):
                raise ValueError("Data too long")
            payload = struct.pack(">HBB", param_id, flags, vtype) + data
            err, _ = self.request(CMD_SET, payload)
            return err

    # ------------- Geolocation -------------

    def geo_request(self) -> None:
        """Sends a geolocation request. There is NO immediate response."""
        self.send_frame(CMD_GEO_REQ, b"")

    def _decode_geo(self, p: bytes) -> Optional[YukiModuleGeo]:
        if len(p) != 22:
            return None
        # Offsets: 0:fix,1:sats,2:ts(4),6:lat(4),10:lon(4),14:alt(4),18:hdop(4)
        fix = p[0]
        sats = p[1]
        ts = struct.unpack(">I", p[2:6])[0]
        lat = struct.unpack(">i", p[6:10])[0]
        lon = struct.unpack(">i", p[10:14])[0]
        alt = struct.unpack(">i", p[14:18])[0]
        hdop = struct.unpack(">I", p[18:22])[0]
        return YukiModuleGeo(fix, sats, ts, lat, lon, alt, hdop)

    # ------------- Polling -------------

    def poll_once(self) -> bool:
        """
        Reads a single incoming frame and processes it.
        Returns True if a frame was processed successfully.
        """
        try:
            t, p = self.recv_frame()
        except TimeoutError:
            return False
        except Exception as ex:
            self.log.error("poll_once: %s", ex)
            return False

        if t == CMD_GEO_RPT and self.geo_callback:
            geo = self._decode_geo(p)
            if geo:
                self.geo_callback(geo)
        elif t == CMD_SET:
            # Optional: incoming SETs could be logged here
            self.log.info("Incoming SET: %s", p.hex())
        else:
            self.log.debug("Unexpected type: 0x%02X (len=%d)", t, len(p))
        return True

    def poll_loop(self, duration: Optional[float] = None) -> None:
        """
        Endless loop (or 'duration' seconds) to receive asynchronous frames.
        """
        start = time.time()
        while True:
            self.poll_once()
            if duration is not None and (time.time() - start) >= duration:
                break

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

# ---------------------------------------------------------------------
# Type helpers for CLI
# ---------------------------------------------------------------------

def parse_type_and_value(tname: str, value: str) -> Tuple[int, bytes]:
    t = tname.lower()
    if t in ("u8", "uint8"):
        return TYPE_UINT8, struct.pack("B", int(value, 0))
    if t in ("i8", "int8"):
        v = int(value, 0)
        if not -128 <= v <= 127:
            raise ValueError("int8 out of range")
        return TYPE_INT8, struct.pack("b", v)
    if t in ("u16", "uint16"):
        return TYPE_UINT16, struct.pack(">H", int(value, 0) & 0xFFFF)
    if t in ("i16", "int16"):
        v = int(value, 0)
        if not -32768 <= v <= 32767:
            raise ValueError("int16 out of range")
        return TYPE_INT16, struct.pack(">h", v)
    if t in ("u32", "uint32"):
        return TYPE_UINT32, struct.pack(">I", int(value, 0) & 0xFFFFFFFF)
    if t in ("i32", "int32"):
        v = int(value, 0)
        if not -2147483648 <= v <= 2147483647:
            raise ValueError("int32 out of range")
        return TYPE_INT32, struct.pack(">i", v)
    if t in ("u64", "uint64"):
        return TYPE_UINT64, struct.pack(">Q", int(value, 0) & 0xFFFFFFFFFFFFFFFF)
    if t in ("i64", "int64"):
        v = int(value, 0)
        if not -9223372036854775808 <= v <= 9223372036854775807:
            raise ValueError("int64 out of range")
        return TYPE_IINT64, struct.pack(">q", v)
    if t in ("bool", "boolean"):
        bv = value.lower()
        vv = 1 if bv in ("1", "true", "yes", "y", "on") else 0
        return TYPE_BOOL, struct.pack("B", vv)
    if t in ("float", "f32"):
        return TYPE_FLOAT, struct.pack(">f", float(value))
    if t in ("double", "f64"):
        return TYPE_DOUBLE, struct.pack(">d", float(value))
    if t in ("string", "str"):
        b = value.encode("utf-8")
        return TYPE_STRING, b
    if t in ("bin", "hex"):
        # value erwartet hex ohne Spaces, z. B. "DEADBEEF"
        v = bytes.fromhex(value.replace(" ", ""))
        return TYPE_BIN, v
    raise ValueError(f"unknown type: {tname}")

# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YUKI-Module Python Client (Debug)")
    ap.add_argument("-p", "--port", required=True, help="serial port (e.g. /dev/ttyUSB0 or COM5)")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baudrate (Default: 115200)")
    ap.add_argument("--timeout", type=float, default=1.0, help="Read/Write timeout in seconds (Default: 1.0)")
    ap.add_argument("-v", "--verbose", action="count", default=0, help="Verbosity (once or multiple times)")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status")
    sub.add_parser("sync")
    sub.add_parser("version")
    sub.add_parser("imei")
    sub.add_parser("iccid")
    sub.add_parser("pubkey")
    sub.add_parser("geo-request")
    poll_p = sub.add_parser("poll")
    poll_p.add_argument("--secs", type=float, default=None, help="Optional limit in seconds")

    set_p = sub.add_parser("set", help="set Value")
    set_p.add_argument("param_id", help="Parameter-ID (e.g. 0x1234)")
    set_p.add_argument("type", help="Datatype (uint8,int16,string,bin,...)")
    set_p.add_argument("value", help="Value (for bin: use Hex, e.g. DEADBEEF)")
    set_p.add_argument("--ro", action="store_true", help="read-only Flag setzen")

    args = ap.parse_args(argv)

    # Logging
    lvl = logging.WARNING
    if args.verbose == 1:
        lvl = logging.INFO
    elif args.verbose >= 2:
        lvl = logging.DEBUG
    logging.basicConfig(level=lvl, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("yuki-module")

    # Callback for geolocation
    def on_geo(g: YukiModuleGeo) -> None:
        lat = g.lat_e7 / 1e7
        lon = g.lon_e7 / 1e7
        alt = g.alt_cm / 100.0
        print(f"GEO: fix={g.fix_type} sats={g.sats} t={g.ts_utc} lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}m hdop={g.hdop_centi/100.0:.2f}")

    try:
        cli = YukiModuleClient(args.port, baud=args.baud, timeout=args.timeout, geo_callback=on_geo, logger=log)
    except Exception as e:
        log.error("Couldn't open port: %s", e)
        return 2

    rc = 0
    try:
        if args.cmd == "status":
            e = cli.status()
            print("STATUS:", ERR_STR.get(e, hex(e)))
        elif args.cmd == "sync":
            e = cli.sync()
            print("SYNC:", ERR_STR.get(e, hex(e)))
        elif args.cmd == "version":
            e, s = cli.version()
            print("VERSION:", ERR_STR.get(e, hex(e)), s if e == ERR_OK else "")
        elif args.cmd == "imei":
            e, s = cli.get_imei()
            print("IMEI:", ERR_STR.get(e, hex(e)), s if e == ERR_OK else "")
        elif args.cmd == "iccid":
            e, s = cli.get_iccid()
            print("ICCID:", ERR_STR.get(e, hex(e)), s if e == ERR_OK else "")
        elif args.cmd == "pubkey":
            e, b = cli.get_pubkey()
            if e == ERR_OK:
                print("PUBKEY:", b.hex())
            else:
                print("PUBKEY:", ERR_STR.get(e, hex(e)))
        elif args.cmd == "geo-request":
            cli.geo_request()
            print("GEO-Request sent. Use 'poll', to receive reports.")
        elif args.cmd == "poll":
            cli.poll_loop(duration=args.secs)
        elif args.cmd == "set":
            try:
                pid = int(args.param_id, 0)
                vtype, data = parse_type_and_value(args.type, args.value)
                e = cli.set_value(pid, vtype, data, read_only=args.ro)
                print("SET:", ERR_STR.get(e, hex(e)))
            except Exception as ex:
                log.error("SET failed: %s", ex)
                rc = 3
        else:
            ap.print_help()
    finally:
        cli.close()

    return rc

if __name__ == "__main__":
    sys.exit(main())
