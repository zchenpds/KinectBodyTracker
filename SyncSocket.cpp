#include "SyncSocket.h"



SyncSocket::SyncSocket() :
	m_socketListen(INVALID_SOCKET),
	m_nPacketCount(0),
	m_nErrorCount(0),
	m_bWs2Loaded(false),
	m_tsWindows(-1),
	m_tsOdroid(-1)
{
}



SyncSocket::~SyncSocket()
{
	releaseResource();
}

bool SyncSocket::init()
{
	int iResult;
	WSADATA wsaData;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
	{
		m_bWs2Loaded = false;
		return false;
	}
	else
		m_bWs2Loaded = true;
	

	// Create a listening socket
	m_socketListen = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (INVALID_SOCKET == m_socketListen)
	{
		releaseResource();
		return false;
	}

	// Set the socket I/O mode: In this case FIONBIO
	// enables or disables the blocking mode for the
	// socket based on the numerical value of iMode.
	// If iMode = 0, blocking is enabled;
	// If iMode != 0, non-blocking mode is enabled.
	u_long iMode = 0;
	iResult = ioctlsocket(m_socketListen, FIONBIO, &iMode);
	if (iResult != NO_ERROR)
	{
		releaseResource();
		return false;
	}

	// Bind the socket to any address and the specified port
	SOCKADDR_IN addrListen;
	
	ZeroMemory(&addrListen, sizeof(addrListen));

	addrListen.sin_family = AF_INET;
	addrListen.sin_addr.s_addr = htonl(INADDR_ANY);
	addrListen.sin_port = htons(SYNC_SOCKET_PORT);

	iResult = bind(m_socketListen, (const sockaddr *)&addrListen, sizeof(addrListen));
	if (iResult != 0) {
		releaseResource();
		return false;
	}


	return true;
}

OdroidTimestamp SyncSocket::receive(INT64 tsWindows, SportSolePacket * pPacket)
{
	int ret;

	sockaddr * addrSource;
	int lenAddrSource;
	SportSolePacket packet;

	ret = recvfrom(m_socketListen, m_pBuffer, PACKET_LENGTH_TIME, 0, addrSource, &lenAddrSource);

	if (ret>1)
	{
		m_nPacketCount++;
		if (checkSportSolePacket((uint8_t *)m_pBuffer))
		{
			reconstructStructSportSolePacket((uint8_t *)m_pBuffer, packet);
			if (pPacket != NULL)
				memcpy(pPacket, &packet, sizeof(SportSolePacket));
			OdroidTimestamp * pOts = (OdroidTimestamp *)packet.Odroid_Timestamp; // assuming big-endian
			m_tsWindows = tsWindows;
			m_tsOdroid = *pOts;
			return *pOts;
		}
		else
		{
			m_nErrorCount++;
		}

	}

	return (OdroidTimestamp)0; // the socket received nothing or some error occurred.
}

bool SyncSocket::checkSportSolePacket(uint8_t * buffer)
{
	return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[PACKET_LENGTH_TIME - 3] == 0x4 && buffer[PACKET_LENGTH_TIME - 2] == 0x5 && buffer[PACKET_LENGTH_TIME - 1] == 0x6) && (buffer[3] == 0x02);
}

void SyncSocket::reconstructStructSportSolePacket(uint8_t * recvbuffer, SportSolePacket & dataPacket)
{
	uint8_t *pointer;

	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];

	// val
	pointer = (uint8_t *)&dataPacket.val;
	pointer[0] = recvbuffer[3];
	//Timestamp_Odroid
	pointer = (uint8_t *)&dataPacket.Odroid_Timestamp;
	pointer[7] = recvbuffer[4];
	pointer[6] = recvbuffer[5];
	pointer[5] = recvbuffer[6];
	pointer[4] = recvbuffer[7];
	pointer[3] = recvbuffer[8];
	pointer[2] = recvbuffer[9];
	pointer[1] = recvbuffer[10];
	pointer[0] = recvbuffer[11];

	// trigger
	pointer = (uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0] = recvbuffer[12];


	//recvbuffer[4];
	//recvbuffer[5];
	//recvbuffer[6];
}

void SyncSocket::releaseResource()
{
	// Ensure WSACleanup() only runs once
	if (m_bWs2Loaded)
	{
		WSACleanup();
		m_bWs2Loaded = false;
	}
}
