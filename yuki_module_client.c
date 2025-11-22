#include "yuki_module_client.h"
#include <string.h>
#include <errno.h>

/* State management */
static struct {
    void*          ctx;
    yuki_module_read_fn   read;
    yuki_module_write_fn  write;
    yuki_module_on_set_fn handler;
    yuki_module_on_geo_fn geo_cb;
    bool           ready;
} g;

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no final XOR) */
static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Header pack/unpack */
static void pack_header(uint8_t* h, uint8_t t, uint16_t l)
{
    h[0] = (uint8_t)((t << 1) | ((l >> 8) & 0x01));
    h[1] = (uint8_t)(l & 0xFF);
}
static void unpack_header(const uint8_t* h, uint8_t* t, uint16_t* l)
{
    *t = (uint8_t)(h[0] >> 1);
    *l = (uint16_t)(((h[0] & 0x01) << 8) | h[1]);
}

/* I/O helpers */
static bool io_read_all(void* ctx, yuki_module_read_fn rf, uint8_t* buf, size_t len)
{
    size_t off = 0;
    while (off < len) {
        ssize_t n = rf(ctx, buf + off, len - off);
        if (n <= 0) return false;
        off += (size_t)n;
    }
    return true;
}
static bool io_write_all(void* ctx, yuki_module_write_fn wf, const uint8_t* buf, size_t len)
{
    size_t off = 0;
    while (off < len) {
        ssize_t n = wf(ctx, buf + off, len - off);
        if (n <= 0) return false;
        off += (size_t)n;
    }
    return true;
}

/* Sending/receiving frames (CRC required) */
static bool send_frame(const YukiModuleMsg* m)
{
    if (m->t > 0x7F || m->l > YUKI_MODULE_MAX_TLV_LENGTH) { errno = EINVAL; return false; }

    uint8_t header[2];
    pack_header(header, m->t, m->l);

    uint8_t tmp[2 + YUKI_MODULE_MAX_TLV_LENGTH];
    memcpy(tmp, header, 2);
    if (m->l) memcpy(tmp + 2, m->v, m->l);
    uint16_t crc = crc16_ccitt_false(tmp, (size_t)(2 + m->l));
    uint8_t crcbe[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };

    if (!io_write_all(g.ctx, g.write, header, 2)) return false;
    if (m->l && !io_write_all(g.ctx, g.write, m->v, m->l)) return false;
    if (!io_write_all(g.ctx, g.write, crcbe, 2)) return false;
    return true;
}
static bool recv_frame(YukiModuleMsg* m)
{
    uint8_t header[2];
    if (!io_read_all(g.ctx, g.read, header, 2)) return false;

    unpack_header(header, &m->t, &m->l);
    if (m->l > YUKI_MODULE_MAX_TLV_LENGTH) { errno = EPROTO; return false; }

    if (m->l && !io_read_all(g.ctx, g.read, m->v, m->l)) return false;

    uint8_t crcbe[2];
    if (!io_read_all(g.ctx, g.read, crcbe, 2)) return false;
    uint16_t crc_rx = (uint16_t)crcbe[0] << 8 | crcbe[1];

    uint8_t tmp[2 + YUKI_MODULE_MAX_TLV_LENGTH];
    memcpy(tmp, header, 2);
    if (m->l) memcpy(tmp + 2, m->v, m->l);
    uint16_t crc_calc = crc16_ccitt_false(tmp, (size_t)(2 + m->l));

    if (crc_calc != crc_rx) { errno = EPROTO; return false; }
    return true;
}

/* Big-endian helpers for 32-bit values */
static uint32_t be32u(const uint8_t* p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static int32_t be32s(const uint8_t* p)
{
    return (int32_t)be32u(p);
}

/* Public API */
bool yuki_module_init(const YukiModuleInterface* iface, void* reserved)
{
    (void)reserved;
    if (!iface || !iface->read || !iface->write) { errno = EINVAL; return false; }
    g.ctx     = iface->ctx;
    g.read    = iface->read;
    g.write   = iface->write;
    g.handler = iface->handler;
    g.geo_cb  = iface->geo_cb;
    g.ready   = true;
    return true;
}

bool yuki_module_send(const YukiModuleMsg* out)
{
    if (!g.ready || !out) { errno = g.ready ? EINVAL : EBUSY; return false; }
    return send_frame(out);
}

bool yuki_module_request(const YukiModuleMsg* out, YukiModuleMsg* in_msg, YukiModuleErr* err)
{
    if (!g.ready || !out) { errno = g.ready ? EINVAL : EBUSY; return false; }

    g.ready = false;

    if (!send_frame(out)) { g.ready = true; return false; }

    YukiModuleMsg resp;
    if (!recv_frame(&resp)) { g.ready = true; return false; }

    if (resp.l == 0) { g.ready = true; errno = EPROTO; return false; }

    if (err) *err = (YukiModuleErr)resp.v[0];
    if (in_msg) *in_msg = resp;

    g.ready = true;
    return true;
}

/* Process incoming frame: SET and GEO_RPT are forwarded to callbacks. */
bool yuki_module_poll_once(void)
{
    YukiModuleMsg m;
    if (!recv_frame(&m)) return false;

    if (m.t == CMD_SET && g.handler) {
        if (m.l < 4) return true;
        uint16_t id  = (uint16_t)((m.v[0] << 8) | m.v[1]);
        bool ro      = (m.v[2] & 0x10) != 0;
        uint8_t type = m.v[3];

        const void* data_ptr = NULL;
        size_t      len      = 0;

        if (type == TYPE_STRING || type == TYPE_BIN) {
            if (m.l < 6) return true;
            len = (size_t)((m.v[4] << 8) | m.v[5]);
            if (m.l != len + 6) return true;
            data_ptr = &m.v[6];
        } else {
            len = (size_t)(m.l - 4);
            data_ptr = &m.v[4];
        }

        g.handler(id, type, ro, data_ptr, len);

        YukiModuleMsg r = { .t = m.t, .l = 1, .v = { (uint8_t)ERR_OK } };
        (void)send_frame(&r);
        return true;
    }

    if (m.t == CMD_GEO_RPT && g.geo_cb) {
        /* Geolocation report – fixed binary format (big-endian):
           Offset | Size  | Field
           0     | 1     | fix_type (0=none,1=2D,2=3D)
           1     | 1     | sats
           2     | 4     | ts_utc
           6     | 4     | lat_e7   (signed)
           10    | 4     | lon_e7   (signed)
           14    | 4     | alt_cm   (signed)
           18    | 4     | hdop*100 (unsigned)
           Total: 22 bytes
        */
        if (m.l == 22) {
            YukiModuleGeo gfix;
            gfix.fix_type   = m.v[0];
            gfix.sats       = m.v[1];
            gfix.ts_utc     = be32u(&m.v[2]);
            gfix.lat_e7     = be32s(&m.v[6]);
            gfix.lon_e7     = be32s(&m.v[10]);
            gfix.alt_cm     = be32s(&m.v[14]);
            gfix.hdop_centi = be32u(&m.v[18]);
            g.geo_cb(&gfix);
        }
        return true;
    }

    return true;
}

/* High-level helpers */
bool yuki_module_set(uint16_t id, uint8_t type, const void* data, size_t len, bool read_only)
{
    YukiModuleMsg m; memset(&m, 0, sizeof m);
    m.t = CMD_SET;

    m.v[0] = (uint8_t)((id >> 8) & 0xFF);
    m.v[1] = (uint8_t)(id & 0xFF);
    m.v[2] = read_only ? 0x10 : 0x00;
    m.v[3] = type;

    if (type == TYPE_STRING || type == TYPE_BIN) {
        if (len > (YUKI_MODULE_MAX_TLV_LENGTH - 6)) { errno = EMSGSIZE; return false; }
        m.v[4] = (uint8_t)((len >> 8) & 0xFF);
        m.v[5] = (uint8_t)(len & 0xFF);
        if (len && data) memcpy(&m.v[6], data, len);
        m.l = (uint16_t)(len + 6);
    } else {
        if (len > (YUKI_MODULE_MAX_TLV_LENGTH - 4)) { errno = EMSGSIZE; return false; }
        if (len && data) memcpy(&m.v[4], data, len);
        m.l = (uint16_t)(len + 4);
    }

    YukiModuleMsg r; YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, &r, &e)) return false;
    return (e == ERR_OK);
}

bool yuki_module_sync(void)
{
    YukiModuleMsg m = { .t = CMD_SYNC, .l = 0 };
    YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, NULL, &e)) return false;
    return (e == ERR_OK);
}

bool yuki_module_version(char* out, size_t out_len)
{
    if (!out || out_len == 0) { errno = EINVAL; return false; }
    YukiModuleMsg m = { .t = CMD_VERSION, .l = 0 };
    YukiModuleMsg r; YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, &r, &e)) return false;
    if (e != ERR_OK) { errno = EPROTO; return false; }

    if (r.l <= 1) { out[0] = '\0'; return true; }
    size_t n = (size_t)(r.l - 1);
    if (n >= (out_len - 1)) n = out_len - 1;
    memcpy(out, &r.v[1], n);
    out[n] = '\0';
    return true;
}

bool yuki_module_status(YukiModuleErr* out_status)
{
    YukiModuleMsg m = { .t = CMD_STATUS, .l = 0 };
    YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, NULL, &e)) return false;
    if (out_status) *out_status = e;
    return true;
}

bool yuki_module_get_pubkey(uint8_t out64[64])
{
    if (!out64) { errno = EINVAL; return false; }
    YukiModuleMsg m = { .t = CMD_GET_PUBKEY, .l = 0 };
    YukiModuleMsg r; YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, &r, &e)) return false;
    if (e != ERR_OK || r.l < (1 + 64)) { errno = EPROTO; return false; }
    memcpy(out64, &r.v[1], 64);
    return true;
}

bool yuki_module_get_imei(char* out, size_t out_len)
{
    if (!out || out_len == 0) { errno = EINVAL; return false; }
    YukiModuleMsg m = { .t = CMD_GET_IMEI, .l = 0 };
    YukiModuleMsg r; YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, &r, &e)) return false;
    if (e != ERR_OK) { errno = EPROTO; return false; }
    if (r.l <= 1) { out[0] = '\0'; return true; }
    size_t n = (size_t)(r.l - 1);
    if (n >= (out_len - 1)) n = out_len - 1;
    memcpy(out, &r.v[1], n);
    out[n] = '\0';
    return true;
}

bool yuki_module_get_iccid(char* out, size_t out_len)
{
    if (!out || out_len == 0) { errno = EINVAL; return false; }
    YukiModuleMsg m = { .t = CMD_GET_ICCID, .l = 0 };
    YukiModuleMsg r; YukiModuleErr e = ERR_INTERNAL;
    if (!yuki_module_request(&m, &r, &e)) return false;
    if (e != ERR_OK) { errno = EPROTO; return false; }
    if (r.l <= 1) { out[0] = '\0'; return true; }
    size_t n = (size_t)(r.l - 1);
    if (n >= (out_len - 1)) n = out_len - 1;
    memcpy(out, &r.v[1], n);
    out[n] = '\0';
    return true;
}

/* Geolocation API */
bool yuki_module_geo_request(void)
{
    YukiModuleMsg m = { .t = CMD_GEO_REQ, .l = 0 };
    /* No synchronous response expected; just send. */
    return yuki_module_send(&m);
}

bool yuki_module_set_geo_callback(yuki_module_on_geo_fn cb)
{
    g.geo_cb = cb;
    return true;
}
