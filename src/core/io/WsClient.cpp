#include "core/io/WsClient.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <random>
#include <sstream>
#include <utility>
#include <vector>

namespace {
std::string base64Encode(const std::uint8_t* data, size_t len) {
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((len + 2) / 3) * 4);

  size_t i = 0;
  while (i + 2 < len) {
    const std::uint32_t v = (static_cast<std::uint32_t>(data[i]) << 16U) |
                            (static_cast<std::uint32_t>(data[i + 1]) << 8U) |
                            static_cast<std::uint32_t>(data[i + 2]);
    out.push_back(table[(v >> 18U) & 0x3F]);
    out.push_back(table[(v >> 12U) & 0x3F]);
    out.push_back(table[(v >> 6U) & 0x3F]);
    out.push_back(table[v & 0x3F]);
    i += 3;
  }

  if (i < len) {
    std::uint32_t v = static_cast<std::uint32_t>(data[i]) << 16U;
    if (i + 1 < len) {
      v |= static_cast<std::uint32_t>(data[i + 1]) << 8U;
    }
    out.push_back(table[(v >> 18U) & 0x3F]);
    out.push_back(table[(v >> 12U) & 0x3F]);
    out.push_back((i + 1 < len) ? table[(v >> 6U) & 0x3F] : '=');
    out.push_back('=');
  }

  return out;
}
}  // namespace

WsClient::WsClient(std::string hostIn, int portIn, std::string pathIn)
    : host(std::move(hostIn)), port(portIn), path(std::move(pathIn)) {}

WsClient::~WsClient() {
  close();
}

bool WsClient::connect() {
  close();

  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* result = nullptr;
  const std::string portStr = std::to_string(port);
  if (getaddrinfo(host.c_str(), portStr.c_str(), &hints, &result) != 0) {
    return false;
  }

  bool connected = false;
  for (auto* rp = result; rp != nullptr; rp = rp->ai_next) {
    const int fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (fd < 0) {
      continue;
    }
    if (::connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
      sockFd = fd;
      connected = true;
      break;
    }
    ::close(fd);
  }
  freeaddrinfo(result);

  if (!connected) {
    return false;
  }

  const std::string key = makeSecWebSocketKey();
  std::ostringstream req;
  req << "GET " << path << " HTTP/1.1\r\n";
  req << "Host: " << host << ':' << port << "\r\n";
  req << "Upgrade: websocket\r\n";
  req << "Connection: Upgrade\r\n";
  req << "Sec-WebSocket-Key: " << key << "\r\n";
  req << "Sec-WebSocket-Version: 13\r\n\r\n";
  const std::string request = req.str();

  if (!sendAll(reinterpret_cast<const std::uint8_t*>(request.data()), request.size())) {
    close();
    return false;
  }

  std::string response;
  if (!readHttpResponse(response)) {
    close();
    return false;
  }
  if (response.find("101") == std::string::npos) {
    close();
    return false;
  }
  return true;
}

bool WsClient::isConnected() const {
  return sockFd >= 0;
}

bool WsClient::sendText(const std::string& payload) {
  if (!isConnected()) {
    return false;
  }

  std::vector<std::uint8_t> frame;
  frame.reserve(payload.size() + 14);
  frame.push_back(0x81);  // FIN + text

  const std::size_t len = payload.size();
  constexpr std::uint8_t maskBit = 0x80;
  if (len <= 125) {
    frame.push_back(maskBit | static_cast<std::uint8_t>(len));
  } else if (len <= 0xFFFF) {
    frame.push_back(maskBit | 126);
    frame.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
    frame.push_back(static_cast<std::uint8_t>(len & 0xFF));
  } else {
    frame.push_back(maskBit | 127);
    for (int i = 7; i >= 0; --i) {
      frame.push_back(static_cast<std::uint8_t>((len >> (i * 8)) & 0xFF));
    }
  }

  std::array<std::uint8_t, 4> mask{};
  std::random_device rd;
  for (auto& b : mask) {
    b = static_cast<std::uint8_t>(rd());
    frame.push_back(b);
  }

  for (std::size_t i = 0; i < len; ++i) {
    frame.push_back(static_cast<std::uint8_t>(payload[i]) ^ mask[i % 4]);
  }

  if (!sendAll(frame.data(), frame.size())) {
    close();
    return false;
  }
  return true;
}

void WsClient::close() {
  if (sockFd >= 0) {
    ::shutdown(sockFd, SHUT_RDWR);
    ::close(sockFd);
    sockFd = -1;
  }
}

bool WsClient::sendAll(const std::uint8_t* data, size_t len) {
  size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::send(sockFd, data + sent, len - sent, 0);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool WsClient::readHttpResponse(std::string& response) {
  response.clear();
  char buf[512];
  while (response.find("\r\n\r\n") == std::string::npos) {
    const ssize_t n = ::recv(sockFd, buf, sizeof(buf), 0);
    if (n <= 0) {
      return false;
    }
    response.append(buf, static_cast<size_t>(n));
    if (response.size() > 8192) {
      return false;
    }
  }
  return true;
}

std::string WsClient::makeSecWebSocketKey() const {
  std::array<std::uint8_t, 16> key{};
  std::random_device rd;
  for (auto& b : key) {
    b = static_cast<std::uint8_t>(rd());
  }
  return base64Encode(key.data(), key.size());
}
