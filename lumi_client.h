#ifndef LUMI_CLIENT_H
#define LUMI_CLIENT_H

/*
 * LUMI UART Client – Benutzer-API
 *
 * Telegrammformat (UART):
 *   Byte 0..1: Header
 *      - Type (7 Bit) linksbündig in Byte 0
 *      - Länge L (9 Bit) = Nutzdatenbytes (ohne CRC)
 *         L[8]  -> Bit0 von Byte 0
 *         L[7:0]-> Byte 1
 *   Byte 2..(2+L-1): Payload (V)
 *   Byte (2+L)..(3+L): CRC-16 (CCITT-FALSE), Big-Endian, berechnet über Header+Payload
 *
 * Hinweise:
 *   - L enthält nur die Länge der Nutzdaten; der CRC gehört nicht zu L.
 *   - Bei CRC-Fehlern wird das Telegramm verworfen; die Funktion liefert false/Fehlercode.
 *   - Asynchrone Ereignisse (z. B. Geolocation-Report) werden über Callbacks signalisiert.
 *
 * Minimalbeispiel:
 *   ssize_t my_read(void* c, uint8_t* b, size_t l);
 *   ssize_t my_write(void* c, const uint8_t* b, size_t l);
 *
 *   void on_set(uint16_t id, uint8_t type, bool ro, const void* data, size_t len);
 *   void on_geo(const struct LumiGeo* g);
 *
 *   LumiInterface ifc = { .ctx = ctx, .read = my_read, .write = my_write,
 *                         .handler = on_set, .geo_cb = on_geo };
 *   lumi_init(&ifc, NULL);
 *   lumi_geo_request();       // Geofix anfordern; Report kommt später via Callback
 *   while (run) { lumi_poll_once(); }
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LUMI_MAX_TLV_LENGTH 511

/* Kommandos (Type) */
typedef enum {
    CMD_GET_PUBKEY = 0x00,
    CMD_GET_IMEI   = 0x01,
    CMD_GET_ICCID  = 0x02,
    CMD_SET        = 0x04,
    CMD_SYNC       = 0x05,
    CMD_VERSION    = 0x06,
    CMD_STATUS     = 0x07,
    CMD_GEO_REQ    = 0x08,
    CMD_GEO_RPT    = 0x09
} LumiCmd;

/* Datentypen im Payload (TYPE_*) */
typedef enum {
    TYPE_INT32   = 0x01,
    TYPE_INT16   = 0x02,
    TYPE_INT8    = 0x03,
    TYPE_UINT32  = 0x04,
    TYPE_UINT16  = 0x05,
    TYPE_UINT8   = 0x06,
    TYPE_BOOL    = 0x07,
    TYPE_UUID    = 0x08,
    TYPE_FLOAT   = 0x09,
    TYPE_DATETIME= 0x0A,
    TYPE_DOUBLE  = 0x0B,
    TYPE_BIN     = 0x0C,
    TYPE_UINT64  = 0x0D,
    TYPE_STRING  = 0x0E,
    TYPE_IINT64  = 0x0F
} LumiType;

/* Fehlercodes im Antwort-Payload[0] bei synchronen Antworten */
typedef enum {
    ERR_OK        = 0x00,
    ERR_CMD       = 0x01,
    ERR_ARG       = 0x02,
    ERR_BUSY      = 0x03,
    ERR_INTERNAL  = 0xFF
} LumiErr;

/* Geolocation-Daten (Report-Payload dekodiert) */
typedef struct {
    /* Fix-Typ: 0 = kein Fix, 1 = 2D, 2 = 3D */
    uint8_t  fix_type;
    /* Anzahl sichtbarer/benutzter Satelliten (je nach Moduldefinition) */
    uint8_t  sats;
    /* UNIX-Zeit in Sekunden (UTC) */
    uint32_t ts_utc;
    /* Breitengrad und Längengrad in 1e-7 Grad (WGS84) */
    int32_t  lat_e7;
    int32_t  lon_e7;
    /* Höhe in Zentimetern (MSL) – kann negativ sein */
    int32_t  alt_cm;
    /* HDOP * 100 (z. B. 95 => 0.95) */
    uint32_t hdop_centi;
} LumiGeo;

/* UART-IO Callback-Signaturen */
typedef ssize_t (*lumi_read_fn)(void* ctx, uint8_t* buf, size_t len);
typedef ssize_t (*lumi_write_fn)(void* ctx, const uint8_t* buf, size_t len);

/* SET-Callback bei eingehenden CMD_SET */
typedef void (*lumi_on_set_fn)(
    uint16_t id,
    uint8_t  type,
    bool     read_only,
    const void* data,
    size_t   len
);

/* Geolocation-Callback bei CMD_GEO_RPT */
typedef void (*lumi_on_geo_fn)(const LumiGeo* geo);

/* UART-Schnittstelle + Callbacks */
typedef struct {
    void*          ctx;
    lumi_read_fn   read;
    lumi_write_fn  write;
    lumi_on_set_fn handler;   /* optional */
    lumi_on_geo_fn geo_cb;    /* optional; wird bei CMD_GEO_RPT aufgerufen */
} LumiInterface;

/* TLV-Container (kann für eigene Helfer benutzt werden) */
typedef struct {
    uint8_t  t;
    uint16_t l;
    uint8_t  v[LUMI_MAX_TLV_LENGTH];
} LumiMsg;

/* Initialisierung – vor allen anderen Aufrufen */
bool lumi_init(const LumiInterface* iface, void* reserved);

/* Einen Eingangsframe verarbeiten (ruft Callbacks bei Bedarf) */
bool lumi_poll_once(void);

/* Generisches Senden/Anfragen */
bool lumi_send(const LumiMsg* out);
bool lumi_request(const LumiMsg* out, LumiMsg* in_msg, LumiErr* err);

/* High-Level API */
bool lumi_set(uint16_t id, uint8_t type, const void* data, size_t len, bool read_only);
bool lumi_sync(void);
bool lumi_version(char* out, size_t out_len);
bool lumi_status(LumiErr* out_status);
bool lumi_get_pubkey(uint8_t out64[64]);
bool lumi_get_imei(char* out, size_t out_len);
bool lumi_get_iccid(char* out, size_t out_len);

/* Geolocation-API */
bool lumi_geo_request(void);                 /* CMD_GEO_REQ senden; Report kommt asynchron */
bool lumi_set_geo_callback(lumi_on_geo_fn);  /* Callback nachträglich setzen/ändern */

#ifdef __cplusplus
}
#endif

#endif /* LUMI_CLIENT_H */
