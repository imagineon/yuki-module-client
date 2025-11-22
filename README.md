# 1. UART Protocol & C API Integration

This document describes the UART protocol (TLV structure with mandatory CRC checking) and the associated C API for integration into your own applications or microcontroller firmware.  
The library abstracts all framing, checksum logic and the processing of incoming events such as geolocation reports.

In addition, the package contains a Python client (`yuki_module_client.py`) which implements the same protocol and provides a convenient CLI for debugging and scripting.

---

## 1.1 Overview

**Protocol properties**

| Property | Description |
|----------|-------------|
| Structure | TLV (Type-Length-Value) |
| Checksum | CRC-16 (CCITT-FALSE, poly 0x1021, init 0xFFFF, no final XOR) |
| Byte order | Big-endian |
| Max. payload | 511 bytes (without CRC) |
| Transport | UART 115200 baud 8N1 |
| Response time | typically < 100 ms (synchronous) |
| Asynchronous events | CMD_SET, CMD_GEO_RPT |

---

## 1.2 Files

| File | Purpose |
|--------|---------|
| `yuki_module_client.h` | Public interface (API, types, functions) |
| `yuki_module_client.c` | Implementation of transport, CRC and event dispatch |
| `yuki_module_client.py` | Python client & CLI (debugging tool and scripting interface) |

---

## 1.3 Integration

This section shows how to integrate the C library into your firmware.  
The Python client is described in section **1.11**.

### 1.3.1 Initialization

Implement UART read/write functions and optional callbacks for received SET values and geolocation reports:

```c
ssize_t uart_read(void* ctx, uint8_t* buf, size_t len);
ssize_t uart_write(void* ctx, const uint8_t* buf, size_t len);

void on_value_changed(uint16_t id, uint8_t type, bool ro,
                      const void* data, size_t len);

void on_geolocation(const YukiModuleGeo* geo);

YukiModuleInterface iface = {
    .ctx     = my_ctx,
    .read    = uart_read,
    .write   = uart_write,
    .handler = on_value_changed,
    .geo_cb  = on_geolocation
};

yuki_module_init(&iface, NULL);
```

**Polling:**  
In your main loop, regularly call `yuki_module_poll_once()` so that incoming frames are processed and callbacks are triggered.

---

### 1.3.2 High-level examples

```c
// Set a simple UINT8 value
uint8_t v = 5;
yuki_module_set(0x1234, TYPE_UINT8, &v, sizeof v, false);

// Synchronise and check status
yuki_module_sync();
YukiModuleErr st;
yuki_module_status(&st);

// Retrieve device information
char imei[32], iccid[32], ver[32];
yuki_module_get_imei(imei, sizeof imei);
yuki_module_get_iccid(iccid, sizeof iccid);
yuki_module_version(ver, sizeof ver);

// Public key (64 B)
uint8_t pubkey[64];
yuki_module_get_pubkey(pubkey);
```

**Asynchronous geolocation:**

```c
// Send request (no immediate response)
yuki_module_geo_request();

// Callback on later report
void on_geolocation(const YukiModuleGeo* g) {
    // g->fix_type (0 = none, 1 = 2D, 2 = 3D)
    // g->lat_e7 / g->lon_e7 (WGS84 × 1e-7)
    // g->alt_cm, g->hdop_centi, g->sats, g->ts_utc
}
```

---

## 1.4 Frame format

```text
[0..1]   Header
   Bit 15–1 : Type (7 bits, left-aligned)
   Bit 0 + byte 1 : Length L (9 bits) = number of payload bytes (without CRC)
[2..(2+L-1)] Payload (V)
[2+L .. 3+L] CRC-16 (CCITT-FALSE, big-endian)
```

**CRC calculation**

```text
crc = 0xFFFF
for each byte in (header + payload):
    crc ^= (byte << 8)
    8 × { crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1 }
```

Length `L` counts **only** the payload; the CRC value itself is not included.

If the CRC check fails, the library automatically discards the frame.

---

## 1.5 Commands (Type)

| Code | Symbol | Direction | Description |
|:----:|:----------------|:--------------|:-------------------------------|
| 0x00 | CMD_GET_PUBKEY | Host → module | Retrieve public key (64 B) |
| 0x01 | CMD_GET_IMEI | Host → module | Retrieve IMEI (ASCII) |
| 0x02 | CMD_GET_ICCID | Host → module | Retrieve ICCID (ASCII) |
| 0x04 | CMD_SET | both directions | Set a value or deliver an event |
| 0x05 | CMD_SYNC | Host → module | Cloud synchronisation |
| 0x06 | CMD_VERSION | Host → module | Firmware version |
| 0x07 | CMD_STATUS | Host → module | Status (error code) |
| 0x08 | CMD_GEO_REQ | Host → module | Request geolocation |
| 0x09 | CMD_GEO_RPT | Module → host | Geolocation report (asynchronous) |

**Synchronous response format:**  
The first byte of the response payload contains the error code (`ERR_OK`, `ERR_CMD`, `ERR_ARG`, `ERR_BUSY`, `ERR_INTERNAL`).  
Additional data starts at `payload[1]`.

---

## 1.6 Data types

| Symbol | Description |
|:--------|:--------------|
| TYPE_INT8/16/32/64 | signed integer |
| TYPE_UINT8/16/32/64 | unsigned integer |
| TYPE_BOOL | Boolean (1 byte) |
| TYPE_FLOAT/DOUBLE | IEEE-754 |
| TYPE_DATETIME | Unix time (UTC) |
| TYPE_STRING | UTF-8 with 16-bit length field |
| TYPE_BIN | Binary data with 16-bit length field |

---

## 1.7 CMD_SET layout

| Offset | Field | Size | Description |
|:------:|:------|:------:|:-------------|
| 0 | ID_H | 1 | Parameter ID high byte |
| 1 | ID_L | 1 | Parameter ID low byte |
| 2 | Flags | 1 | Bit 4 = read-only |
| 3 | Type | 1 | Data type (TYPE_*) |
| 4–5 | Len | 2 | Only for STRING/BIN: length |
| 6–n | Data | n | Payload |

For non-STRING/BIN types the `Len` field is omitted; the data begins at offset 4.

---

## 1.8 Asynchronous geolocation

**Sequence**

1. Host sends `CMD_GEO_REQ`
2. Module determines position (GNSS / cellular)
3. When a fix is available, the module sends `CMD_GEO_RPT`

**Report payload (22 bytes, big-endian)**

| Offset | Field | Size | Unit / meaning |
|:------:|:------|:------:|:--------------------|
| 0 | fix_type | 1 | 0 = no fix, 1 = 2D, 2 = 3D |
| 1 | sats | 1 | satellites used |
| 2 | ts_utc | 4 | Unix time (UTC) |
| 6 | lat_e7 | 4 | latitude × 1e-7 (WGS-84) |
| 10 | lon_e7 | 4 | longitude × 1e-7 (WGS-84) |
| 14 | alt_cm | 4 | altitude (cm MSL) |
| 18 | hdop_centi | 4 | HDOP × 100 |

The library automatically decodes this structure and calls the registered callback:

```c
void on_geolocation(const YukiModuleGeo* g);
```

---

## 1.9 Error handling

| Category | Behaviour |
|------------|------------|
| CRC error | Frame is discarded |
| Invalid length / ID | `errno = EPROTO` |
| UART timeout | depends on host driver |
| Error code in payload | see `YukiModuleErr` |

---

## 1.10 Compatibility

- CRC checking is **mandatory** (older frames without CRC are not accepted).
- Protocol versions ≥ 2.x support geolocation (`CMD_GEO_*`).
- Firmware version can be queried via `CMD_VERSION`.

---

## 1.11 Python client (`yuki_module_client.py`)

In addition to the C API, the repository contains a Python 3 client that implements the same UART TLV protocol and CRC logic. It can be used both as a command-line tool and as a library.

**Installation**

The only external dependency is `pyserial`:

```bash
pip install pyserial
```

**Command-line usage**

Typical calls:

```bash
python yuki_module_client.py --port /dev/ttyUSB0 status
python yuki_module_client.py -p /dev/ttyUSB0 version
python yuki_module_client.py -p /dev/ttyUSB0 imei
python yuki_module_client.py -p /dev/ttyUSB0 iccid
python yuki_module_client.py -p /dev/ttyUSB0 pubkey
python yuki_module_client.py -p /dev/ttyUSB0 geo-request
python yuki_module_client.py -p /dev/ttyUSB0 poll --secs 30
```

Available subcommands include (excerpt):

- `status` – query the current status of the module
- `sync` – trigger a cloud synchronisation
- `version` – read the firmware version
- `imei`, `iccid`, `pubkey` – read IMEI, ICCID and the 64-byte public key
- `geo-request` – send a geolocation request (`CMD_GEO_REQ`)
- `poll` – continuously read and decode frames (including `CMD_GEO_RPT`)
- `set` – write a value by parameter ID and type

All commands use the same error codes as the C library (`ERR_OK`, `ERR_CMD`, …) and print them in a human-readable form.

**Using the Python API in your own scripts**

You can also import the client and control the module programmatically:

```python
from yuki_module_client import YukiModuleClient, TYPE_UINT8

# Open serial port
cli = YukiModuleClient(port="/dev/ttyUSB0", baud=115200)

# Synchronise with the cloud
err = cli.sync()
print("SYNC:", err)

# Read basic information
err, version = cli.version()
print("VERSION:", err, version)

err, imei = cli.get_imei()
err, iccid = cli.get_iccid()

# Write a UINT8 value to parameter 0x1234
value = (5).to_bytes(1, "big")
err = cli.set_value(0x1234, TYPE_UINT8, value, read_only=False)
print("SET:", err)

# Request geolocation and poll for reports
cli.geo_request()
cli.poll_loop(duration=10.0)

cli.close()
```

For asynchronous geolocation reports, you can pass a `geo_callback` when constructing `YukiModuleClient`; it will then be called with a decoded `YukiModuleGeo` instance whenever a `CMD_GEO_RPT` frame is received.
