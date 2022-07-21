/**
*	WEART - Common TCP Client
*	https://www.weart.it/
*/

#define WIN32_LEAN_AND_MEAN

#include <iostream>

#include "pch.h"
#include <WEART_SDK/WeArtClient.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>

#include <ppltasks.h>
using namespace Windows::Foundation;
using namespace concurrency;
using namespace Windows::Devices::Enumeration;

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define DEFAULT_BUFLEN 512
int recvbuflen = DEFAULT_BUFLEN;
char recvbuf[DEFAULT_BUFLEN];

#define DATA_BUFSIZE 4096

static char messagesSeparator = '~';

// https://docs.microsoft.com/en-us/windows/win32/winsock/complete-client-code
// reference
WeArtClient::WeArtClient(PCSTR ip_address, PCSTR port) {

    IP_ADDESS = ip_address;
    PORT = port;
}

void WeArtClient::Run() {

    WSADATA wsaData;
    ConnectSocket = INVALID_SOCKET;
    struct addrinfo* result = NULL,
        * ptr = NULL,
        hints;

    int iResult;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        IsConnected = false;
        printf("WSAStartup failed with error: %d\n", iResult);
        return;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(IP_ADDESS, PORT, &hints, &result);
    if (iResult != 0) {
        IsConnected = false;
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return;
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            IsConnected = false;
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return;
        }

        // Connect to server.
        iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            IsConnected = false;
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        IsConnected = false;
        WSACleanup();
        return;
    }

    IsConnected = true;
    
    std::cout << "Calling Windows::System::Threading::WorkItemHandler lambda function ... " << std::endl;

    // receive data in background 
    auto workItem = ref new Windows::System::Threading::WorkItemHandler([this](IAsyncAction^ workItem)
        {
            std::cout << "Calling OnReceive() ... " << std::endl;
            OnReceive();
        });

    std::cout << "Calling Windows::System::Threading::ThreadPool::RunAsync ... " << std::endl;
    auto asyncAction = Windows::System::Threading::ThreadPool::RunAsync(workItem);
}


void WeArtClient::OnReceive() {

    WSABUF DataBuf;

    char buffer[DATA_BUFSIZE];

    DWORD EventTotal = 0, RecvBytes = 0, Flags = 0;
    DWORD BytesTransferred;

    WSAEVENT EventArray[WSA_MAXIMUM_WAIT_EVENTS];

    WSAOVERLAPPED AcceptOverlapped;

    std::string trailingText;

    EventArray[EventTotal] = WSACreateEvent();

    ZeroMemory(&AcceptOverlapped, sizeof(WSAOVERLAPPED));

    AcceptOverlapped.hEvent = EventArray[EventTotal];

    DataBuf.len = DATA_BUFSIZE;
    DataBuf.buf = buffer;

    EventTotal++;

    if (WSARecv(ConnectSocket, &DataBuf, 1, &RecvBytes, &Flags, &AcceptOverlapped, NULL) == SOCKET_ERROR)
    {
        if (WSAGetLastError() != WSA_IO_PENDING)

        {
            printf("Error reading buffer socket");
        }
    }

    while (TRUE)
    {
        DWORD Index;

        Index = WSAWaitForMultipleEvents(EventTotal, EventArray, FALSE, WSA_INFINITE, FALSE);

        WSAResetEvent(EventArray[Index - WSA_WAIT_EVENT_0]);

        WSAGetOverlappedResult(ConnectSocket, &AcceptOverlapped, &BytesTransferred, FALSE, &Flags);

        if (BytesTransferred == 0)
        {
            printf("Closing socket %d\n", ConnectSocket);
            closesocket(ConnectSocket);
            WSACloseEvent(EventArray[Index - WSA_WAIT_EVENT_0]);

            return;
        }

        std::string bufferText = std::string(buffer);

        const int lastSeparatorIndex = bufferText.find_last_of(messagesSeparator);
        trailingText = bufferText.substr(lastSeparatorIndex + 1);
        std::string textToParse = bufferText.substr(0, lastSeparatorIndex);

        // Split the string on separator occurence
        std::vector<std::string> splitStrings;
        std::istringstream dataStream(textToParse);
        std::string s;
        while (getline(dataStream, s, messagesSeparator)) {
            if (s.empty())
                continue;
            splitStrings.push_back(s);
        }

        std::vector<WeArtMessage*> messages;

        // Std::strings to WeArtMessages
        messages.resize(splitStrings.size());
        for (int i = 0; i < messages.size(); i++) {
            messages[i] = messageSerializer.Deserialize(splitStrings[i]);
        }

        TrackingMessages(messages);

        Flags = 0;

        ZeroMemory(&AcceptOverlapped, sizeof(WSAOVERLAPPED));

        AcceptOverlapped.hEvent = EventArray[Index - WSA_WAIT_EVENT_0];

        DataBuf.len = DATA_BUFSIZE;
        DataBuf.buf = buffer;

        if (WSARecv(ConnectSocket, &DataBuf, 1, &RecvBytes, &Flags, &AcceptOverlapped, NULL) == SOCKET_ERROR)
        {
            if (WSAGetLastError() != WSA_IO_PENDING)
            {
                printf("Error reading buffer socket");
            }
        }
    }
}

void WeArtClient::Start() {
    StartFromClientMessage startMsg;
    SendMessage(&startMsg);
}

void WeArtClient::Stop() {
    StopFromClientMessage stopMsg;
    SendMessage(&stopMsg);
}

void WeArtClient::Close() {

    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;

    // shutdown the connection since no more data will be sent
    int iResult = shutdown(ConnectSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return;
    }

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

    IsConnected = false;
}

void WeArtClient::SendMessage(WeArtMessage* message) {

    if (!IsConnected) {
        return;
    }

    // WeArt message to string
    std::string text = messageSerializer.Serialize(message);
    text += messagesSeparator;

    char sendbuf[120];
    strcpy(sendbuf, text.c_str());

    // Send an initial buffer
    int iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
    if (iResult == SOCKET_ERROR) {
        IsConnected = false;
        printf("send failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return;
    }
}

void WeArtClient::TrackingMessages(std::vector<WeArtMessage*> messages)
{
    for (auto msg : messages) {

        if (msg == NULL)
            continue;

        // Forward the message to relevant tracking objects
        for (WeArtThimbleTrackingObject* obj : thimbleTrackingObjects) {
            obj->OnMessageReceived(msg);
        }
    }
}

int WeArtClient::SizeThimbles() {
    return thimbleTrackingObjects.size();
}

void WeArtClient::AddThimbleTracking(WeArtThimbleTrackingObject* trackingObjects) {

    thimbleTrackingObjects.push_back(trackingObjects);
}
