# 1. UART-Protokoll & C-API-Integration

Dieses Document beschreibt das UART-Protokoll (TLV-Struktur mit verpflichtender CRC-Prüfung) und die zugehörige C-API für die Integration in eigene Anwendungen oder Mikrocontroller-Firmware.  
Die Bibliothek abstrahiert das komplette Framing, die Prüfsummen-Logik und die Verarbeitung eingehender Ereignisse wie Geolocation-Reports.

---

## 1.1 Überblick

**Protokoll-Eigenschaften**

| Merkmal | Beschreibung |
|----------|---------------|
| Struktur | TLV (Type-Length-Value) |
| Prüfsumme | CRC-16 (CCITT-FALSE, Poly 0x1021, Init 0xFFFF, kein Final-XOR) |
| Byte-Reihenfolge | Big-Endian |
| Max. Nutzlast | 511 Byte (ohne CRC) |
| Transport | UART 115 200 Bd 8N1 |
| Antwortzeit | typ. < 100 ms (synchron) |
| Asynchrone Ereignisse | CMD_SET, CMD_GEO_RPT |

---

## 1.2 Dateiumfang

| Datei | Zweck |
|--------|--------|
| `lumi_client.h` | öffentliche Schnittstelle (API, Typen, Funktionen) |
| `lumi_client.c` | Implementierung von Transport, CRC und Ereignis-Dispatch |

---

## 1.3 Integration

### 1.3.1 Initialisierung

Implementieren Sie UART-Read/Write-Funktionen und optional Callbacks für empfangene SET-Werte und Geolocation-Reports:

```c
ssize_t uart_read(void* ctx, uint8_t* buf, size_t len);
ssize_t uart_write(void* ctx, const uint8_t* buf, size_t len);

void on_value_changed(uint16_t id, uint8_t type, bool ro,
                      const void* data, size_t len);

void on_geolocation(const LumiGeo* geo);

LumiInterface iface = {
    .ctx     = my_ctx,
    .read    = uart_read,
    .write   = uart_write,
    .handler = on_value_changed,
    .geo_cb  = on_geolocation
};

lumi_init(&iface, NULL);
```

**Polling:**  
In der Hauptschleife regelmäßig `lumi_poll_once()` aufrufen, damit eingehende Frames verarbeitet und Callbacks ausgelöst werden.

---

### 1.3.2 High-Level-Beispiele

```c
// Einfachen UINT8-Wert setzen
uint8_t v = 5;
lumi_set(0x1234, TYPE_UINT8, &v, sizeof v, false);

// Synchronisation und Status prüfen
lumi_sync();
LumiErr st;
lumi_status(&st);

// Geräteinformationen abrufen
char imei[32], iccid[32], ver[32];
lumi_get_imei(imei, sizeof imei);
lumi_get_iccid(iccid, sizeof iccid);
lumi_version(ver, sizeof ver);

// Public-Key (64 B)
uint8_t pubkey[64];
lumi_get_pubkey(pubkey);
```

**Asynchrone Geolocation:**

```c
// Anfrage senden (keine Sofortantwort)
lumi_geo_request();

// Callback bei späterem Report
void on_geolocation(const LumiGeo* g) {
    // g->fix_type (0=kein,1=2D,2=3D)
    // g->lat_e7/g->lon_e7 (WGS84 × 1e-7)
    // g->alt_cm, g->hdop_centi, g->sats, g->ts_utc
}
```

---

## 1.4 Telegrammformat

```
[0..1]   Header
   Bit 15-1 : Type (7 Bit, linksbündig)
   Bit 0 + Byte 1 : Länge L (9 Bit) = Anzahl Payload-Bytes (ohne CRC)
[2..(2+L-1)] Payload (V)
[2+L .. 3+L] CRC-16 (CCITT-FALSE, Big-Endian)
```

**CRC-Berechnung**

```
crc = 0xFFFF
für jedes Byte aus (Header + Payload):
    crc ^= (byte << 8)
    8 × { crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1 }
```

Länge `L` zählt **nur** die Nutzdaten, der CRC-Wert ist nicht enthalten.

Bei CRC-Fehlern verwirft die Bibliothek das Telegramm automatisch.

---

## 1.5 Kommandos (Type)

| Code | Symbol | Richtung | Beschreibung |
|:----:|:----------------|:--------------|:-------------------------------|
| 0x00 | CMD_GET_PUBKEY | Host → Modul | Öffentlichen Schlüssel (64 B) |
| 0x01 | CMD_GET_IMEI | Host → Modul | IMEI abrufen (ASCII) |
| 0x02 | CMD_GET_ICCID | Host → Modul | ICCID abrufen (ASCII) |
| 0x04 | CMD_SET | beidseitig | Wert setzen oder Ereignis übermitteln |
| 0x05 | CMD_SYNC | Host → Modul | Cloud-Synchronisation |
| 0x06 | CMD_VERSION | Host → Modul | Firmware-Version |
| 0x07 | CMD_STATUS | Host → Modul | Status (Fehlercode) |
| 0x08 | CMD_GEO_REQ | Host → Modul | Geolocation anfordern |
| 0x09 | CMD_GEO_RPT | Modul → Host | Geolocation-Report (asynchron) |

**Synchrones Antwortformat:**  
Erstes Byte der Antwort-Payload enthält den Fehlercode (`ERR_OK`, `ERR_CMD`, `ERR_ARG`, `ERR_BUSY`, `ERR_INTERNAL`).  
Weitere Daten beginnen ab `Payload[1]`.

---

## 1.6 Datentypen

| Symbol | Beschreibung |
|:--------|:--------------|
| TYPE_INT8/16/32/64 | signed Integer |
| TYPE_UINT8/16/32/64 | unsigned Integer |
| TYPE_BOOL | Boolean (1 Byte) |
| TYPE_FLOAT/DOUBLE | IEEE-754 |
| TYPE_DATETIME | Unix-Zeit (UTC) |
| TYPE_STRING | UTF-8 mit Längenfeld (16 Bit) |
| TYPE_BIN | Binärdaten mit Längenfeld (16 Bit) |

---

## 1.7 Aufbau CMD_SET

| Offset | Feld | Größe | Beschreibung |
|:------:|:------|:------:|:-------------|
| 0 | ID_H | 1 | Parameter-ID High |
| 1 | ID_L | 1 | Parameter-ID Low |
| 2 | Flags | 1 | Bit4 = read-only |
| 3 | Typ | 1 | Datentyp (TYPE_*) |
| 4–5 | Len | 2 | nur bei STRING/BIN: Länge |
| 6–n | Data | n | Nutzdaten |

Für nicht-STRING/BIN-Typen entfällt `Len`, die Daten beginnen bei Offset 4.

---

## 1.8 Asynchrone Geolocation

**Ablauf**

1. Host sendet `CMD_GEO_REQ`
2. Modul bestimmt Position (GNSS / Mobilfunk)
3. Bei verfügbarem Fix sendet das Modul `CMD_GEO_RPT`

**Report-Payload (22 Byte, Big-Endian)**

| Offset | Feld | Größe | Einheit / Bedeutung |
|:------:|:------|:------:|:--------------------|
| 0 | fix_type | 1 | 0 = kein Fix, 1 = 2D, 2 = 3D |
| 1 | sats | 1 | verwendete Satelliten |
| 2 | ts_utc | 4 | Unix-Zeit (UTC) |
| 6 | lat_e7 | 4 | Breite × 1e-7 (WGS-84) |
| 10 | lon_e7 | 4 | Länge × 1e-7 (WGS-84) |
| 14 | alt_cm | 4 | Höhe (cm MSL) |
| 18 | hdop_centi | 4 | HDOP × 100 |

Die Bibliothek dekodiert diese Struktur automatisch und ruft den registrierten Callback:

```c
void on_geolocation(const LumiGeo* g);
```

---

## 1.9 Fehlerbehandlung

| Kategorie | Verhalten |
|------------|------------|
| CRC-Fehler | Frame wird verworfen |
| Ungültige Länge / ID | errno = EPROTO |
| Timeout UART | abhängig vom Host-Treiber |
| Fehlercode im Payload | siehe LumiErr |

---

## 1.10 Kompatibilität

- CRC-Prüfung ist **verpflichtend** (ältere Frames werden nicht akzeptiert).
- Protokollversionen ≥ 2.x unterstützen Geolocation (CMD_GEO_*).
- Firmware-Version über CMD_VERSION abrufbar.
