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
import cmd
import shlex
import traceback
import logging
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Optional, Tuple

try:
    import serial  # pyserial
except ImportError as e:
    print("pyserial not installed. Please install with 'pip install pyserial'.", file=sys.stderr)
    raise

# Client library version — tracks the YUKI module protocol/firmware it targets
__version__ = "1.0.1"

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
CMD_GEO_ENA    = 0x08
CMD_GEO_RPT    = 0x09
CMD_GET_TIME      = 0x0A
CMD_SET_UUID      = 0x0B
CMD_GET_CLAIMCODE = 0x0D
CMD_GET_LTE_QUALITY     = 0x0E
CMD_GET_LTE_CONNECTED   = 0x0F
CMD_GET_CLOUD_CONNECTED = 0x10
CMD_FACTORY_RESET       = 0x11

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
TYPE_INT64   = 0x0F

# Error codes
ERR_OK        = 0x00
ERR_CMD       = 0x01
ERR_ARG       = 0x02
ERR_BUSY      = 0x03
ERR_SIM       = 0x10
ERR_NET       = 0x11
ERR_CONN      = 0x12
ERR_INTERNAL  = 0xFF

ERR_STR = {
    ERR_OK: "OK",
    ERR_CMD: "CMD",
    ERR_ARG: "ARG",
    ERR_BUSY: "BUSY",
    ERR_SIM: "SIM ERROR",
    ERR_NET: "NOT REGISTERED",
    ERR_CONN: "NOT CONNECTED",
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
        if len(data) != 32:
            self.log.warning("PubKey-lenght unexpected: %d", len(data))
        return err, data

    def get_claimcode(self) -> Tuple[int, str]:
        err, data = self.request(CMD_GET_CLAIMCODE)
        return err, data.decode('utf-8', errors='replace')

    def get_lte_quality(self) -> Tuple[int, Optional[int]]:
        err, data = self.request(CMD_GET_LTE_QUALITY)
        if err != ERR_OK:
            return err, None
        if len(data) < 1:
            raise IOError("Protocol error: CMD_GET_LTE_QUALITY payload too short")
        return err, data[0]

    def get_lte_connected(self) -> Tuple[int, Optional[bool]]:
        err, data = self.request(CMD_GET_LTE_CONNECTED)
        if err != ERR_OK:
            return err, None
        if len(data) < 1:
            raise IOError("Protocol error: CMD_GET_LTE_CONNECTED payload too short")
        return err, data[0] != 0

    def get_cloud_connected(self) -> Tuple[int, Optional[bool]]:
        err, data = self.request(CMD_GET_CLOUD_CONNECTED)
        if err != ERR_OK:
            return err, None
        if len(data) < 1:
            raise IOError("Protocol error: CMD_GET_CLOUD_CONNECTED payload too short")
        return err, data[0] != 0

    def get_time(self) -> Tuple[int, Optional[int]]:
        err, data = self.request(CMD_GET_TIME)
        if err != ERR_OK:
            return err, None
        if len(data) < 4:
            raise IOError(f"Protocol error: CMD_GET_TIME payload too short ({len(data)} bytes)")
        (ts,) = struct.unpack(">I", data[:4])
        return err, ts

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

    def set_uuid(self, value: int) -> int:
        """Sends CMD_SET_UUID"""
        payload = struct.pack(">I", value & 0xFFFFFFFF)
        err, _ = self.request(CMD_SET_UUID, payload)
        return err

    def factory_reset(self) -> bool:
        """Sends CMD_FACTORY_RESET: the module wipes its cached identity
        (IMEI/ICCID/UUID) and reboots without replying, so this is
        fire-and-forget - it only sends the frame and does NOT wait for a
        response (there is none)."""
        self.send_frame(CMD_FACTORY_RESET, b"")
        return True
    # ------------- Geolocation -------------

    def geo_enable(self, enable: int) -> None:
        """Sends a geolocation request. There is NO immediate response."""
        if enable not in (0, 1):
            raise ValueError("enable must be 0 or 1")
        err, _ = self.request(CMD_GEO_ENA, bytes([enable]))
        return err


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
        return TYPE_INT64, struct.pack(">q", v)
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

class CLIError(Exception):
    """Expected user input / usage error (no stack trace unless --debug)."""


def try_parse_type_and_value(tname: str, value: str) -> Tuple[Optional[Tuple[int, bytes]], str]:
    """
    Wrapper around parse_type_and_value that never raises.
    Returns ((vtype, data), "") on success, or (None, "reason") on error.
    """
    try:
        vtype, data = parse_type_and_value(tname, value)
        return (vtype, data), ""
    except Exception as ex:
        return None, str(ex)


def _print_err(msg: str) -> None:
    print(f"Error: {msg}", file=sys.stderr)


def _print_info(msg: str) -> None:
    print(msg)


def _format_pubkey(pk: bytes) -> str:
    # 32 bytes => 64 hex chars
    hx = pk.hex()
    if not hx:
        return ""
    # group into 32-char chunks for readability
    return "\n".join(hx[i:i+32] for i in range(0, len(hx), 32))


class YukiShell(cmd.Cmd):
    intro = (
        "YUKI-Module interactive shell. Type 'help' to list commands.\n"
        "Tip: 'poll 10' keeps reading for 10 seconds.\n"
    )
    prompt = "yuki> "

    def __init__(self, cli: YukiModuleClient, log: logging.Logger, debug: bool = False) -> None:
        super().__init__()
        self.cli = cli
        self.log = log
        self.debug = debug

    # ----- helpers -----

    def _safe_call(self, fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except (TimeoutError, IOError, OSError) as ex:
            _print_err(str(ex))
        except KeyboardInterrupt:
            _print_err("Interrupted.")
        except Exception as ex:
            _print_err(str(ex))
            if self.debug:
                traceback.print_exc()
        return None

    def emptyline(self) -> bool:
        return False  # do nothing on empty line

    def default(self, line: str) -> None:
        _print_err(f"Unknown command: {line.strip()!r}. Type 'help'.")

    # ----- basic -----

    def do_exit(self, arg: str) -> bool:
        """Exit the shell."""
        return True

    def do_quit(self, arg: str) -> bool:
        """Exit the shell."""
        return True

    def do_EOF(self, arg: str) -> bool:
        """Exit the shell (Ctrl-D)."""
        _print_info("")  # newline
        return True

    # ----- module commands -----

    def do_status(self, arg: str) -> None:
        """status  -> query module status"""
        r = self._safe_call(self.cli.status)
        if r is not None:
            _print_info(f"STATUS: {ERR_STR.get(r, hex(r))}")

    def do_sync(self, arg: str) -> None:
        """sync  -> perform sync command"""
        r = self._safe_call(self.cli.sync)
        if r is not None:
            _print_info(f"SYNC: {ERR_STR.get(r, hex(r))}")

    def do_version(self, arg: str) -> None:
        """version  -> get firmware version"""
        r = self._safe_call(self.cli.version)
        if r is not None:
            e, v = r
            if e == ERR_OK:
                _print_info(f"VERSION: {v}")
            else:
                _print_info(f"VERSION: {ERR_STR.get(e, hex(e))}")

    def do_imei(self, arg: str) -> None:
        """imei  -> get IMEI"""
        r = self._safe_call(self.cli.get_imei)
        if r is not None:
            e, v = r
            _print_info(f"IMEI: {v if e == ERR_OK else ERR_STR.get(e, hex(e))}")

    def do_iccid(self, arg: str) -> None:
        """iccid  -> get ICCID"""
        r = self._safe_call(self.cli.get_iccid)
        if r is not None:
            e, v = r
            _print_info(f"ICCID: {v if e == ERR_OK else ERR_STR.get(e, hex(e))}")

    def do_pubkey(self, arg: str) -> None:
        """pubkey  -> get public key"""
        r = self._safe_call(self.cli.get_pubkey)
        if r is not None:
            e, pk = r
            if e == ERR_OK:
                _print_info("PUBKEY:\n" + _format_pubkey(pk))
            else:
                _print_info(f"PUBKEY: {ERR_STR.get(e, hex(e))}")

    def do_claimcode(self, arg: str) -> None:
        """claimcode  -> get claim code"""
        r = self._safe_call(self.cli.get_claimcode)
        if r is not None:
            e, v = r
            _print_info(f"CLAIMCODE: {v if e == ERR_OK else ERR_STR.get(e, hex(e))}")

    def do_lte_quality(self, arg: str) -> None:
        """lte_quality  -> get LTE signal quality (0..255)"""
        r = self._safe_call(self.cli.get_lte_quality)
        if r is not None:
            e, q = r
            _print_info(f"LTE_QUALITY: {q if e == ERR_OK else ERR_STR.get(e, hex(e))}")

    def do_lte_connected(self, arg: str) -> None:
        """lte_connected  -> check if LTE is connected"""
        r = self._safe_call(self.cli.get_lte_connected)
        if r is not None:
            e, v = r
            _print_info(f"LTE_CONNECTED: {v if e == ERR_OK else ERR_STR.get(e, hex(e))}")

    def do_cloud_connected(self, arg: str) -> None:
        """cloud_connected  -> check if cloud is connected"""
        r = self._safe_call(self.cli.get_cloud_connected)
        if r is not None:
            e, v = r
            _print_info(f"CLOUD_CONNECTED: {v if e == ERR_OK else ERR_STR.get(e, hex(e))}")

    def do_time(self, arg: str) -> None:
        """time  -> get device time"""
        r = self._safe_call(self.cli.get_time)
        if r is not None:
            e, ts = r
            if e == ERR_OK and ts is not None:
                _print_info("TIME: " + str(datetime.fromtimestamp(ts)))
            else:
                _print_info(f"TIME: {ERR_STR.get(e, hex(e))}")

    def do_set_uuid(self, arg: str) -> None:
        """set_uuid <value> -> set UUID (uint32)"""
        a = arg.strip()
        if not a:
            _print_err("Usage: set_uuid <value>")
            return
        try:
            value = int(a, 0)
        except ValueError:
            _print_err(f"Inavalid value: {a!r}")
            return
        r = self._safe_call(self.cli.set_uuid, value)
        if r is not None:
            _print_info(f"SET_UUID: {ERR_STR.get(r, hex(r))}")


    def do_factory_reset(self, arg: str) -> None:
        """factory_reset  -> wipe cached identity (IMEI/ICCID/UUID) + reboot (no response expected)"""
        r = self._safe_call(self.cli.factory_reset)
        if r:
            _print_info("FACTORY_RESET: sent (module wipes identity and reboots; no response expected)")

    def do_geo_enable(self, arg: str) -> None:
        """geo-enable <0|1>  -> enable/disable geolocation (async; use poll to receive reports)"""
        a = arg.strip()
        if a not in ("0", "1"):
            _print_err("Usage: geo-enable <0|1>")
            return
        enable = int(a)
        r = self._safe_call(self.cli.geo_enable, enable)
        if r is not None:
            _print_info(f"GEO {'enabled. Use `poll` to receive reports.' if enable else 'disabled'}")

    def do_poll(self, arg: str) -> None:
        """poll [seconds]  -> read incoming frames; prints GEO reports"""
        secs = None
        a = arg.strip()
        if a:
            try:
                secs = float(a)
                if secs <= 0:
                    raise ValueError()
            except Exception:
                _print_err("Usage: poll [seconds]  (seconds must be > 0)")
                return
        self._safe_call(self.cli.poll_loop, duration=secs)

    def do_set(self, arg: str) -> None:
        """set <param_id> <type> <value> [--ro]"""
        try:
            parts = shlex.split(arg)
        except Exception as ex:
            _print_err(f"Could not parse arguments: {ex}")
            return

        if not parts or len(parts) < 3:
            _print_err("Usage: set <param_id> <type> <value> [--ro]")
            return

        ro = False
        if "--ro" in parts:
            ro = True
            parts = [p for p in parts if p != "--ro"]

        if len(parts) < 3:
            _print_err("Usage: set <param_id> <type> <value> [--ro]")
            return

        pid_s, tname, value = parts[0], parts[1], " ".join(parts[2:])

        try:
            pid = int(pid_s, 0)
        except Exception:
            _print_err(f"Invalid param_id: {pid_s!r} (use e.g. 0x1234 or 4660)")
            return

        parsed, err = try_parse_type_and_value(tname, value)
        if parsed is None:
            _print_err(err or "Invalid type/value")
            return
        vtype, data = parsed

        r = self._safe_call(self.cli.set_value, pid, vtype, data, ro)
        if r is not None:
            _print_info(f"SET: {ERR_STR.get(r, hex(r))}")


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="YUKI-Module Python Client (Debug)")
    ap.add_argument("-p", "--port", required=True, help="serial port (e.g. /dev/ttyUSB0 or COM5)")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baudrate (Default: 115200)")
    ap.add_argument("--timeout", type=float, default=1.0, help="Read/Write timeout in seconds (Default: 1.0)")
    ap.add_argument("-v", "--verbose", action="count", default=0, help="Verbosity (once or multiple times)")
    ap.add_argument("--debug", action="store_true", help="Show stack traces for unexpected errors")
    ap.add_argument("--version", action="version", version=f"yuki_module_client {__version__}")

    sub = ap.add_subparsers(dest="cmd")

    sub.add_parser("shell", help="interactive mode (default)")
    sub.add_parser("status")
    sub.add_parser("sync")
    sub.add_parser("version")
    sub.add_parser("imei")
    sub.add_parser("iccid")
    sub.add_parser("pubkey")
    sub.add_parser("claimcode")
    sub.add_parser("lte_quality")
    sub.add_parser("lte_connected")
    sub.add_parser("cloud_connected")
    sub.add_parser("time")
    geo_p = sub.add_parser("geo_enable", help="enable/disable geolocation")
    geo_p.add_argument("enable", type=int, choices=[0, 1], help="1=enable, 0=disable")

    poll_p = sub.add_parser("poll")
    poll_p.add_argument("--secs", type=float, default=None, help="Optional limit in seconds")

    set_uuid_p = sub.add_parser("set_uuid", help="set UUID (uint32)")
    set_uuid_p.add_argument("value", help="uint32 value (e.g. 0x12345678 or 305419896)")

    sub.add_parser("factory_reset", help="wipe cached identity (IMEI/ICCID/UUID) + reboot (no response)")

    set_p = sub.add_parser("set", help="set Value")
    set_p.add_argument("param_id", help="Parameter-ID (e.g. 0x1234)")
    set_p.add_argument("type", help="Datatype (uint8,int16,string,bin,...)")
    set_p.add_argument("value", help="Value (for bin: use Hex, e.g. DEADBEEF)")
    set_p.add_argument("--ro", action="store_true", help="read-only Flag setzen")

    return ap


def run_one_shot(cli: YukiModuleClient, args: argparse.Namespace, log: logging.Logger, debug: bool) -> int:
    """
    Executes exactly one command. Must NOT raise.
    """
    try:
        if args.cmd == "status":
            e = cli.status()
            print("STATUS:", ERR_STR.get(e, hex(e)))
            return 0

        if args.cmd == "sync":
            e = cli.sync()
            print("SYNC:", ERR_STR.get(e, hex(e)))
            return 0

        if args.cmd == "version":
            e, v = cli.version()
            print("VERSION:", v if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "imei":
            e, v = cli.get_imei()
            print("IMEI:", v if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "iccid":
            e, v = cli.get_iccid()
            print("ICCID:", v if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "pubkey":
            e, pk = cli.get_pubkey()
            if e == ERR_OK:
                print("PUBKEY:\n" + _format_pubkey(pk))
                return 0
            print("PUBKEY:", ERR_STR.get(e, hex(e)))
            return 3

        if args.cmd == "claimcode":
            e, v = cli.get_claimcode()
            print("CLAIMCODE:", v if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "lte_quality":
            e, q = cli.get_lte_quality()
            print("LTE_QUALITY:", q if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "lte_connected":
            e, v = cli.get_lte_connected()
            print("LTE_CONNECTED:", v if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "cloud_connected":
            e, v = cli.get_cloud_connected()
            print("CLOUD_CONNECTED:", v if e == ERR_OK else ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "time":
            e, ts = cli.get_time()
            if e == ERR_OK and ts is not None:
                print("TIME:", datetime.fromtimestamp(ts))
                return 0
            print("TIME:", ERR_STR.get(e, hex(e)))
            return 3

        if args.cmd == "geo_enable":
            cli.geo_enable(args.enable)
            print(f"GEO {'enabled' if args.enable else 'disabled'}. Use 'poll' to receive reports.")
            return 0

        if args.cmd == "set_uuid":
            try:
                value = int(args.value, 0)
            except ValueError:
                _print_err(f"Invalid value: {args.value!r}")
                return 2
            e = cli.set_uuid(value)
            print("SET_UUID:", ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        if args.cmd == "factory_reset":
            cli.factory_reset()
            print("FACTORY_RESET: sent (module wipes identity and reboots; no response expected)")
            return 0

        if args.cmd == "poll":
            cli.poll_loop(duration=args.secs)
            return 0

        if args.cmd == "set":
            try:
                pid = int(args.param_id, 0)
            except Exception:
                _print_err(f"Invalid param_id: {args.param_id!r} (use e.g. 0x1234 or 4660)")
                return 2

            parsed, perr = try_parse_type_and_value(args.type, args.value)
            if parsed is None:
                _print_err(perr or "Invalid type/value")
                return 2
            vtype, data = parsed
            e = cli.set_value(pid, vtype, data, read_only=args.ro)
            print("SET:", ERR_STR.get(e, hex(e)))
            return 0 if e == ERR_OK else 3

        _print_err("No command given. Use 'shell' or run with -h for help.")
        return 2

    except KeyboardInterrupt:
        _print_err("Interrupted.")
        return 130
    except (TimeoutError, IOError, OSError) as ex:
        _print_err(str(ex))
        return 3
    except Exception as ex:
        _print_err(str(ex))
        if debug:
            traceback.print_exc()
        return 3


def main(argv=None) -> int:
    ap = build_arg_parser()
    args = ap.parse_args(argv)

    # Logging
    lvl = logging.WARNING
    if args.verbose == 1:
        lvl = logging.INFO
    elif args.verbose >= 2:
        lvl = logging.DEBUG
    logging.basicConfig(level=lvl, format="%(levelname)s: %(message)s")
    log = logging.getLogger("yuki")

    def on_geo(g: YukiModuleGeo) -> None:
        lat = g.lat_e7 / 1e7
        lon = g.lon_e7 / 1e7
        alt = g.alt_cm / 100.0
        print(
            f"GEO: fix={g.fix_type} sats={g.sats} ts={g.ts_utc} "
            f"lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}m hdop={g.hdop_centi/100.0:.2f}"
        )

    try:
        cli = YukiModuleClient(args.port, baud=args.baud, timeout=args.timeout, geo_callback=on_geo, logger=log)
    except Exception as e:
        _print_err(f"Couldn't open port: {e}")
        if args.debug:
            traceback.print_exc()
        return 2

    try:
        # Default to interactive shell if no command or 'shell'
        if not args.cmd or args.cmd == "shell":
            try:
                YukiShell(cli, log=log, debug=args.debug).cmdloop()
                return 0
            except KeyboardInterrupt:
                _print_err("Interrupted.")
                return 130

        return run_one_shot(cli, args, log, debug=args.debug)

    finally:
        cli.close()


if __name__ == "__main__":
    sys.exit(main())
