#pragma once
#include <netinet/in.h>
struct ClientAddress
{
    socklen_t addressLength;
    struct sockaddr_in address;
};