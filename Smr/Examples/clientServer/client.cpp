/// \ingroup Examples
/// \file client.cpp
/// \brief This program demonstrate the usage of SMR lirbary in a basic client/server erchitecture

#include <stdio.h>
#include <stdlib.h>
#include "SmrSTL.h"

#ifdef WIN32
  #include <winsock2.h>
  #include <windows.h>
#else
  #include <sys/socket.h>
  //#include <netdb.h>
  #include <arpa/inet.h>
  #include <sys/types.h>
  #include <netdb.h>
  #include <sys/types.h>
  #include <netinet/in.h>
#endif
#include <string.h>


#define MSG_LENGTH 256
#define PORT 5555
#define IP "127.0.0.1"

int main (int argc, char * argv[])
{
  int sock = 0;
  struct sockaddr_in addr;
#ifdef WIN32
  WSADATA wsaData;
  unsigned long ip;
#else
  struct in_addr ip;
#endif
  memset(&addr,0,sizeof(addr));

#ifdef WIN32

    //---------------------------------------------
    // Initialize Winsock
    WSAStartup(MAKEWORD(2,2), &wsaData);

	ip = inet_addr(IP);
	if (ip == INADDR_NONE){
	  printf("Error while retrieving host name");
	  exit(1);
	}
#else
  if ( (inet_aton(IP, &ip)) == 0){
    printf("Error while retrieving host name");
    exit(1);
  }
#endif

#ifdef WIN32
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if ( sock == INVALID_SOCKET) {
    printf("Error while creating socket");
    exit(1);
  }
#else
  if ( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("Error while creating socket");
    exit(1);
  }
#endif

#ifdef WIN32
  addr.sin_addr.S_un.S_addr = ip;
#else
  addr.sin_addr = ip;
#endif
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);

  ifstream inFile("embotcommands.txt");

  if(!inFile)
  {
    cout << "Cannot open input file embotCommands.txt" << endl;
    exit(1);
  }

  string nextCommand("");

  while(inFile)
  {
    //inFile >> nextCommand;
    //std::stringstream textToSend;
    char buffer[MSG_LENGTH] = "";
    inFile.getline(buffer, 256);
    cout << buffer << endl;
    //strncpy(buffer,textToSend.str().c_str(),textToSend.str().length());
    if (sendto(sock, buffer, MSG_LENGTH, 0, (struct sockaddr*) & addr, sizeof(struct sockaddr_in)) == -1){
      printf("Error while sending socket");
      exit(1);
    }
    Sleep(10);
  }

  inFile.close();

  closesocket(sock);
  WSACleanup();
  exit(0);
}