#pragma once
#ifndef __UDP__H__
#define __UDP__H__

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <stdio.h>  
#include <WINSOCK2.H>  
#include <string>
#include <cstring>
#include <iostream>
#pragma comment(lib,"WS2_32.lib")  
#define BUF_SIZE    80
// #define POST		12345

bool InitUDP (SOCKET& socketSrv,int POST);
void SendAngle(const std::string &Message, const SOCKET&socketSrv, SOCKADDR_IN&addrClient);
//void SendAngle(const std::string &Message, const SOCKET&socketSrv, SOCKADDR_IN& addrClient, const char * strIP, int POST);


#endif // !__UDP__H__

