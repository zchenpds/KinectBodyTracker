#pragma once

#ifndef UNICODE
#define UNICODE
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdint>

const unsigned int SYNC_SOCKET_PORT = 3464;
const unsigned int PACKET_LENGTH_TIME = 16;

#pragma comment(lib, "Ws2_32.lib")

typedef INT64 OdroidTimestamp;

struct SportSolePacket
{
	uint8_t val;
	uint8_t Odroid_Timestamp[8];
	uint8_t Odroid_Trigger;
};

class SyncSocket
{
private:
	SOCKET             m_socketListen;
	WSAEVENT           m_hEventRecv;
	int                m_nPacketCount;
	int                m_nErrorCount;
	bool               m_bWs2Loaded; // indicates whether WSACleanup() is needed on exit
	bool               m_bInitSucceeded;
public:
	INT64              m_tsWindows;
	OdroidTimestamp    m_tsOdroid;
	
public:
	SyncSocket();
	~SyncSocket();
	bool init(HWND hWnd);
	OdroidTimestamp receive(SportSolePacket * pPacket = NULL);
protected:
	bool checkSportSolePacket(uint8_t * buffer);
	void reconstructStructSportSolePacket(uint8_t * recvbuffer, SportSolePacket & dataPacket);
	void releaseResource();
};
