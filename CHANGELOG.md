# Changelog

All notable changes to the YUKI module client are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
The version number tracks the YUKI module firmware / UART protocol version the
client targets.

## [1.0.2] - 2026-07-20

Response-handling fixes. No protocol change and no new commands.

### Fixed
- Both clients now check that a reply echoes the request type before accepting
  it. The module echoes the type back, but neither client compared it, so a
  single read timeout (or any stray frame) made every later request read the
  *previous* request's reply -- silently, for the rest of the session. A
  `lte_quality` call then reported the first ASCII digit of a stale `imei`
  reply, e.g. 56 for an IMEI starting with "8". Unsolicited `GEO_RPT` / `SET`
  frames arriving mid-exchange are dispatched to their callbacks instead of
  being mistaken for the reply.
- Both clients resynchronise after a failed exchange: Python calls
  `reset_input_buffer()`, the C client drains through its read callback (the io
  abstraction has no flush). Without this the stale bytes desynced the next
  request too.
- C `yuki_module_geo_enable()` sent `GEO_ENA` fire-and-forget, but the module
  always answers it with a status byte. The unread reply stayed in the stream
  and shifted every subsequent exchange by one -- a far more reliable trigger
  for the desync above than a timeout. It now reads its reply.
- Python `geo_enable` reported success regardless of the module's answer, in
  both the interactive console and the one-shot CLI. On a build without GNSS
  (EG912) the module answers `ERR_CMD` and the client printed
  "GEO enabled. Use `poll` to receive reports."

### Changed
- C `yuki_module_geo_enable()` now returns `true` only if the module accepted
  the command, not merely if the frame went out. On an EG912 (no GNSS) it
  therefore returns `false` where it previously returned `true`. Callers that
  branch on it will see the difference.

## [1.0.1] - 2026-07-14

Aligns the C and Python clients with the YUKI module 1.0.x UART protocol
(coldwave-yuki-module 1.0.0-RC1).

### Added
- `CMD_FACTORY_RESET` (0x11): `yuki_module_factory_reset()` (C) and
  `factory_reset()` plus a `factory_reset` CLI subcommand (Python). Fire-and-forget —
  the module wipes its cached identity (IMEI/ICCID/UUID) and reboots without replying.
- `ERR_SIM` (0x10), `ERR_NET` (0x11), `ERR_CONN` (0x12) error codes in the C client
  (the Python client already had them).
- Version identifiers: `YUKI_MODULE_CLIENT_VERSION` (C), `__version__` and a
  `--version` CLI flag (Python).

### Fixed
- C client `yuki_module_get_pubkey()` now reads the 32-byte ed25519 key. It
  previously required/copied 64 bytes and therefore failed with `EPROTO` against
  firmware 1.0.x (the 32-byte change had only landed in the Python client).
- `TYPE_INT64` (0x0F) was misspelled `TYPE_IINT64` in both clients.

### Changed
- Python constant `CMD_GPS_ENABLE` renamed to `CMD_GEO_ENA` (0x08) to match the
  module and the C client. Value and wire behaviour are unchanged.
- README updated to the 1.0.x protocol (32-byte public key, full command table
  `0x00`–`0x11`, `CMD_GEO_ENA` naming, extended error codes, corrected examples).

## [1.0.0] - 2025-12

Initial release.

### Added
- C client (`yuki_module_client.c` / `.h`) and Python client
  (`yuki_module_client.py`) for the YUKI module UART TLV protocol.
- Framing with a 2-byte header (7-bit type + 9-bit length, max 511-byte payload)
  and mandatory CRC-16/CCITT-FALSE over header + payload.
- Synchronous request/response helpers and asynchronous handling of incoming
  `CMD_SET` and `CMD_GEO_RPT` (geolocation) frames via callbacks.
- Commands `0x00`–`0x10`: get pubkey / IMEI / ICCID, set, sync, version, status,
  geolocation enable + report, get time, set UUID, get claim code, LTE quality,
  LTE connected, cloud connected.
- Python debug CLI with an interactive shell.
