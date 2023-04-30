#pragma once

#include <netinet/in.h>

#include <string>


namespace vision {
namespace network {
class Sender {
 public:
    Sender(const std::string& ip, int port);
    ~Sender();

    void Send(std::string buffer);
 private:
    int _connect;
    sockaddr_in _addr{};
};
}  // namespace receiver
}  // namespace vision