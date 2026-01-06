#pragma once
#include <WiFiUdp.h>
#include "RobotLink.h"

// UDP Transport wrapper for RobotLink
// Allows RobotLink to work over UDP instead of just Serial
namespace RobotLink {

class UDPStream : public Stream {
public:
  UDPStream(WiFiUDP& udp) : _udp(udp), _hasClient(false) {}

  // Set the client address (call this when you receive a packet)
  void setClient(IPAddress ip, uint16_t port) {
    _clientIP = ip;
    _clientPort = port;
    _hasClient = true;
  }

  // Check if we have a client to send to
  bool hasClient() const { return _hasClient; }

  // Get client info
  IPAddress getClientIP() const { return _clientIP; }
  uint16_t getClientPort() const { return _clientPort; }

  // Stream interface - Reading
  int available() override {
    // First check if there's data remaining in the current packet buffer
    int remaining = _udp.available();
    if (remaining > 0) {
      return remaining;
    }

    // No data remaining from current packet, check for new packet
    // IMPORTANT: parsePacket() should only be called when buffer is empty
    int size = _udp.parsePacket();
    if (size > 0) {
      // Store client address for responses
      _clientIP = _udp.remoteIP();
      _clientPort = _udp.remotePort();
      _hasClient = true;

      // After parsePacket(), call available() again to get the actual buffer size
      return _udp.available();
    }
    return 0;
  }

  int read() override {
    return _udp.read();
  }

  int peek() override {
    return _udp.peek();
  }

  // Stream interface - Writing
  size_t write(uint8_t byte) override {
    if (!_hasClient) return 0;

    // Buffer the byte
    if (_writeBuffer.size() < 256) {
      _writeBuffer.push_back(byte);
    }

    // For UDP, we want each RobotLink frame in its own UDP packet
    // Don't wait for 64 bytes - let the sender call flush() explicitly
    // (or flush will happen on next frame's SOF bytes)

    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    if (!_hasClient) return 0;

    // For bulk writes, send immediately
    _udp.beginPacket(_clientIP, _clientPort);
    size_t written = _udp.write(buffer, size);
    _udp.endPacket();
    return written;
  }

  void flush() override {
    if (!_hasClient || _writeBuffer.empty()) return;

    _udp.beginPacket(_clientIP, _clientPort);
    _udp.write(_writeBuffer.data(), _writeBuffer.size());
    _udp.endPacket();
    _writeBuffer.clear();
  }

private:
  WiFiUDP& _udp;
  IPAddress _clientIP;
  uint16_t _clientPort;
  bool _hasClient;
  std::vector<uint8_t> _writeBuffer;
};

} // namespace RobotLink
