#include "hwa_ins_coder_udpclient.h"
#include <ws2tcpip.h>
#include <iostream>

#pragma comment(lib, "ws2_32.lib")

ins_udp_client::ins_udp_client(const std::string& ip, int port) {
    WSADATA wsaData;
    int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (wsa_result != 0) {
        std::cerr << "WSAStartup failed with error: " << wsa_result << std::endl;
        sock_ = INVALID_SOCKET;
        return;
    }

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) {
        std::cerr << "socket() failed with error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return;
    }
    memset(&target_, 0, sizeof(target_));
    target_.sin_family = AF_INET;
    target_.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &target_.sin_addr);
}

ins_udp_client::~ins_udp_client() {
    closesocket(sock_);
    WSACleanup();
}

bool ins_udp_client::send(const char* data, int length) {
    return sendto(sock_, data, length, 0, (sockaddr*)&target_, sizeof(target_)) != SOCKET_ERROR;
}