#pragma once

#include <cstdint>
#include <string>

class WsClient {
 public:
  WsClient(std::string host, int port, std::string path);
  ~WsClient();

  bool connect();
  bool isConnected() const;
  bool sendText(const std::string& payload);
  void close();

 private:
  bool sendAll(const std::uint8_t* data, size_t len);
  bool readHttpResponse(std::string& response);
  std::string makeSecWebSocketKey() const;

  std::string host;
  int port;
  std::string path;
  int sockFd{-1};
};

