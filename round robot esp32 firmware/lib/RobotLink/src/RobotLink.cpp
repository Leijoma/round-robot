#include "RobotLink.h"
#include <stdlib.h>

namespace RobotLink {

static inline uint16_t crc16_update_ccitt_false(uint16_t crc, uint8_t byte) {
  crc ^= (uint16_t)byte << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  return crc;
}

Link::Link(Stream& io, const Config& cfg)
  : _io(io), _cfg(cfg) {
  // allocate payload buffer
  uint8_t cap = _cfg.maxPayload;
  if (cap == 0) cap = 1;
  _buf = (uint8_t*)malloc(cap);
  resetRx();
}

void Link::resetRx() {
  _st = S_WAIT_SOF0;
  _type = 0;
  _len = 0;
  _idx = 0;
  _rxCrc = 0;
  _calcCrc = 0;
}

void Link::dropByte() {
  _bytesDropped++;
}

uint16_t Link::crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc = crc16_update_ccitt_false(crc, data[i]);
  }
  return crc;
}

bool Link::sendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  // Header
  _io.write(SOF0);
  _io.write(SOF1);
  _io.write(type);
  _io.write(len);

  // CRC over type,len,payload
  uint16_t crc = 0xFFFF;
  crc = crc16_update_ccitt_false(crc, type);
  crc = crc16_update_ccitt_false(crc, len);
  for (uint8_t i = 0; i < len; i++) {
    _io.write(payload[i]);
    crc = crc16_update_ccitt_false(crc, payload[i]);
  }

  _io.write((uint8_t)(crc & 0xFF));
  _io.write((uint8_t)((crc >> 8) & 0xFF));

  // Flush to ensure frame is sent immediately (important for UDP)
  _io.flush();

  return true;
}

bool Link::poll(FrameHandler handler) {
  bool delivered = false;

  while (_io.available() > 0) {
    int b = _io.read();
    if (b < 0) break;
    uint8_t by = (uint8_t)b;

    switch (_st) {
      case S_WAIT_SOF0:
        if (by == SOF0) _st = S_WAIT_SOF1;
        else dropByte();
        break;

      case S_WAIT_SOF1:
        if (by == SOF1) _st = S_READ_TYPE;
        else { dropByte(); resetRx(); }
        break;

      case S_READ_TYPE:
        _type = by;
        _calcCrc = 0xFFFF;
        _calcCrc = crc16_update_ccitt_false(_calcCrc, _type);
        _st = S_READ_LEN;
        break;

      case S_READ_LEN:
        _len = by;
        _calcCrc = crc16_update_ccitt_false(_calcCrc, _len);

        if (_cfg.rejectOversize && _len > _cfg.maxPayload) {
          _framesBadLen++;
          // discard upcoming payload+crc bytes by resetting and resyncing
          resetRx();
          break;
        }

        _idx = 0;
        if (_len == 0) _st = S_READ_CRC_LO;
        else _st = S_READ_PAYLOAD;
        break;

      case S_READ_PAYLOAD:
        if (_idx < _cfg.maxPayload) {
          _buf[_idx] = by;
        }
        _calcCrc = crc16_update_ccitt_false(_calcCrc, by);
        _idx++;

        if (_idx >= _len) {
          _st = S_READ_CRC_LO;
        }
        break;

      case S_READ_CRC_LO:
        _rxCrc = (uint16_t)by;
        _st = S_READ_CRC_HI;
        break;

      case S_READ_CRC_HI:
        _rxCrc |= ((uint16_t)by << 8);

        if (_rxCrc == _calcCrc) {
          _framesOk++;
          if (handler) {
            handler(_type, _buf, _len);
            delivered = true;
          }
        } else {
          _framesBadCrc++;
        }

        resetRx();
        break;

      default:
        resetRx();
        break;
    }
  }

  return delivered;
}

// Little-endian helpers
void Link::wr_u16_le(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); }
void Link::wr_i16_le(uint8_t* p, int16_t v)  { wr_u16_le(p, (uint16_t)v); }
void Link::wr_u32_le(uint8_t* p, uint32_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF); }
void Link::wr_i32_le(uint8_t* p, int32_t v)  { wr_u32_le(p, (uint32_t)v); }

uint16_t Link::rd_u16_le(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
int16_t  Link::rd_i16_le(const uint8_t* p)  { return (int16_t)rd_u16_le(p); }
uint32_t Link::rd_u32_le(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
int32_t  Link::rd_i32_le(const uint8_t* p)  { return (int32_t)rd_u32_le(p); }

} // namespace RobotLink
