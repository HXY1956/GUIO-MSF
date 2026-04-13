#pragma once
#define _WINSOCKAPI_  
#include <winsock2.h>
#include <string>

class ins_udp_client {
public:
    ins_udp_client(const std::string& ip, int port);
    ~ins_udp_client();

    bool send(const char* data, int length);

private:
    SOCKET sock_;
    sockaddr_in target_;
};
