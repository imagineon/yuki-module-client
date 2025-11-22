#ifndef YUKI_MODULE_CLIENT_H
#define YUKI_MODULE_CLIENT_H

/*
 * YUKI-Module UART Client – User API
 *
 * Telegram frame format (UART):
 *   Byte 0..1: Header
 *      - Type (7 bits), left-aligned in byte 0
 *      - Length L (9 bits) = payload bytes (without CRC)
 *         L[8]   -> bit 0 of byte 0
 *         L[7:0] -> byte 1
 *   Byte 2..(2+L-1): Payload (V)
 *   Byte (2+L)..(3+L): CRC-16 (CCITT-FALSE), big-endian, calculated over header+payload
 *
 * Notes:
 *   - L contains only the length of the payload; the CRC is not part of L.
 *   - On CRC errors, the frame is discarded; the function returns false/an error code.
 *   - Asynchronous events (e.g. geolocation reports) are signalled via callbacks.
 *
 * Minimal example:
 *   ssize_t my_read(void* c, uint8_t* b, size_t l);
 *   ssize_t my_write(void* c, const uint8_t* b, size_t l);
 *
 *   void on_set(uint16_t id, uint8_t type, bool ro, const void* data, size_t len);
 *   void on_geo(const struct YukiModuleGeo* g);
 *
 *   YukiModuleInterface ifc = { .ctx = ctx, .read = my_read, .write = my_write,
 *                         .handler = on_set, .geo_cb = on_geo };
 *   yuki_module_init(&ifc, NULL);
 *   yuki_module_geo_request();       // Request geo fix; report arrives later via callback
 *   while (run) { yuki_module_poll_once(); }
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LUMI_MAX_TLV_LENGTH 511

/* Commands (Type) */
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
} YukiModuleCmd;

/* Data types in payload (TYPE_*) */
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
} YukiModuleType;

/* Error codes in response payload[0] for synchronous replies */
typedef enum {
    ERR_OK        = 0x00,
    ERR_CMD       = 0x01,
    ERR_ARG       = 0x02,
    ERR_BUSY      = 0x03,
    ERR_INTERNAL  = 0xFF
} YukiModuleErr;

/* Geolocation data (decoded report payload) */
typedef struct {
    /* Fix type: 0 = no fix, 1 = 2D, 2 = 3D */
    uint8_t  fix_type;
    /* Number of visible/used satellites (depending on module definition) */
    uint8_t  sats;
    /* UNIX time in seconds (UTC) */
    uint32_t ts_utc;
    /* Latitude and longitude in 1e-7 degrees (WGS84) */
    int32_t  lat_e7;
    int32_t  lon_e7;
    /* Altitude in centimetres (MSL) – can be negative */
    int32_t  alt_cm;
    /* HDOP * 100 (e.g. 95 => 0.95) */
    uint32_t hdop_centi;
} YukiModuleGeo;

/* UART I/O callback signatures */
typedef ssize_t (*yuki_module_read_fn)(void* ctx, uint8_t* buf, size_t len);
typedef ssize_t (*yuki_module_write_fn)(void* ctx, const uint8_t* buf, size_t len);

/* SET callback for incoming CMD_SET */
typedef void (*yuki_module_on_set_fn)(
    uint16_t id,
    uint8_t  type,
    bool     read_only,
    const void* data,
    size_t   len
);

/* Geolocation callback for CMD_GEO_RPT */
typedef void (*yuki_module_on_geo_fn)(const YukiModuleGeo* geo);

/* UART interface + callbacks */
typedef struct {
    void*          ctx;
    yuki_module_read_fn   read;
    yuki_module_write_fn  write;
    yuki_module_on_set_fn handler;   /* optional */
    yuki_module_on_geo_fn geo_cb;    /* optional; called on CMD_GEO_RPT */
} YukiModuleInterface;

/* TLV container (can be used for custom helpers) */
typedef struct {
    uint8_t  t;
    uint16_t l;
    uint8_t  v[LUMI_MAX_TLV_LENGTH];
} YukiModuleMsg;

/* Initialisation – must be called before all other functions */
bool yuki_module_init(const YukiModuleInterface* iface, void* reserved);

/* Process one incoming frame (invokes callbacks as needed) */
bool yuki_module_poll_once(void);

/* Generic send/request helpers */
bool yuki_module_send(const YukiModuleMsg* out);
bool yuki_module_request(const YukiModuleMsg* out, YukiModuleMsg* in_msg, YukiModuleErr* err);

/* High-level API */
bool yuki_module_set(uint16_t id, uint8_t type, const void* data, size_t len, bool read_only);
bool yuki_module_sync(void);
bool yuki_module_version(char* out, size_t out_len);
bool yuki_module_status(YukiModuleErr* out_status);
bool yuki_module_get_pubkey(uint8_t out64[64]);
bool yuki_module_get_imei(char* out, size_t out_len);
bool yuki_module_get_iccid(char* out, size_t out_len);

/* Geolocation API */
bool yuki_module_geo_request(void);                 /* Send CMD_GEO_REQ; report is delivered asynchronously */
bool yuki_module_set_geo_callback(lumi_on_geo_fn);  /* Set or change callback afterwards */

#ifdef __cplusplus
}
#endif

#endif /* LUMI_CLIENT_H */
