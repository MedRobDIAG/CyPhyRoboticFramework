#include "serial.h"

#ifdef _WIN32
    Serial::Serial(const wchar_t *portName)
    {
        //We're not yet connected
        this->connected = false;

        //Try to connect to the given port throuh CreateFile
        this->hSerial = CreateFile(portName,
                GENERIC_READ | GENERIC_WRITE,
                0,
                NULL,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                NULL);

        //Check if the connection was successfull
        if(this->hSerial==INVALID_HANDLE_VALUE)
        {
            //If not success full display an Error
            if(GetLastError()==ERROR_FILE_NOT_FOUND){

                //Print Error if neccessary
                printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

            }
            else
            {
                printf("ERROR!!!");
            }
        }
        else
        {
            //If connected we try to set the comm parameters
            DCB dcbSerialParams = {0};

            //Try to get the current
            if (!GetCommState(this->hSerial, &dcbSerialParams))
            {
                //If impossible, show an error
                printf("failed to get current serial parameters!");
            }
            else
            {
                //Define serial connection parameters for the arduino board
                dcbSerialParams.BaudRate=CBR_9600;
                dcbSerialParams.ByteSize=8;
                dcbSerialParams.StopBits=ONESTOPBIT;
                dcbSerialParams.Parity=NOPARITY;

                 //Set the parameters and check for their proper application
                 if(!SetCommState(hSerial, &dcbSerialParams))
                 {
                    printf("ALERT: Could not set Serial Port parameters");
                 }
                 else
                 {
                     //If everything went fine we're connected
                     this->connected = true;
                     //We wait 2s as the arduino board will be reseting
                     Sleep(2000);
                 }
            }
        }

    }

    Serial::~Serial()
    {
        //Check if we are connected before trying to disconnect
        if(this->connected)
        {
            //We're no longer connected
            this->connected = false;
            //Close the serial handler
            CloseHandle(this->hSerial);
        }
    }

    int Serial::ReadData(char *buffer, unsigned int nbChar)
    {
        //Number of bytes we'll have read
        DWORD bytesRead;
        //Number of bytes we'll really ask to read
        unsigned int toRead;

        //Use the ClearCommError function to get status info on the Serial port
        ClearCommError(this->hSerial, &this->errors, &this->status);

        //Check if there is something to read
        if(this->status.cbInQue>0)
        {
            //If there is we check if there is enough data to read the required number
            //of characters, if not we'll read only the available characters to prevent
            //locking of the application.
            if(this->status.cbInQue>nbChar)
            {
                toRead = nbChar;
            }
            else
            {
                toRead = this->status.cbInQue;
            }

            //Try to read the require number of chars, and return the number of read bytes on success
            if(ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
            {
                return bytesRead;
            }

        }

        //If nothing has been read, or that an error was detected return -1
        return -1;

    }


    bool Serial::WriteData(char *buffer, unsigned int nbChar)
    {
        DWORD bytesSend;

        //Try to write the buffer on the Serial port
        if(!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
        {
            //In case it don't work get comm error and return false
            ClearCommError(this->hSerial, &this->errors, &this->status);

            return false;
        }
        else
            return true;
    }

    bool Serial::IsConnected()
    {
        //Simply return the connection status
        return this->connected;

    }
#else
    #include <string.h>   /* String function definitions */
    #include <fcntl.h>    /* File control definitions */
    #include <iostream>

    int serialport_writebyte( int fd, uint8_t b)
    {
        int n = write(fd,&b,1);
        if( n!=1)
            return -1;
        return 0;
    }

    int serialport_write(int fd, const char* str)
    {
        int len = strlen(str);
        int n = write(fd, str, len);
        if( n!=len ) 
            return -1;
        return 0;
    }

    int serialport_read_until(int fd, char* buf, char until)
    {
        char b[1];
        int i=0;
        do { 
            int n = read(fd, b, 1);  // read a char at a time
            if( n==0 || n==-1 ) {
                usleep( 10 * 1000 ); // wait 10 msec try again
                continue;
            }
            buf[i] = b[0]; i++;
        } while( b[0] != until );
        
        buf[i] = 0;  // null terminate the string
        return 0;
    }

    // takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
    // and a baud rate (bps) and connects to that port at that speed and 8N1.
    // opens the port in fully raw mode so you can send binary data.
    // returns valid fd, or -1 on error
    int serialport_init(const char* serialport)
    {
        struct termios toptions;
        int fd;
        
        fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)  {
            perror("init_serialport: Unable to open port ");
            return -1;
        }
        
        if (tcgetattr(fd, &toptions) < 0) {
            perror("init_serialport: Couldn't get term attributes");
            return -1;
        }
        
        int baud(9600);

        speed_t brate = baud; // let you override switch below if needed
        switch(baud) {
            case 4800:   brate=B4800;   break;
            case 9600:   brate=B9600;   break;
    #ifdef B14400
            case 14400:  brate=B14400;  break;
    #endif
            case 19200:  brate=B19200;  break;
    #ifdef B28800
            case 28800:  brate=B28800;  break;
    #endif
            case 38400:  brate=B38400;  break;
            case 57600:  brate=B57600;  break;
            case 115200: brate=B115200; break;
        }
        cfsetispeed(&toptions, brate);
        cfsetospeed(&toptions, brate);

        
        // rs232
        toptions.c_cflag = brate | CS8 | CLOCAL | CREAD;
        toptions.c_iflag = IGNPAR;
        toptions.c_oflag = 0;
        toptions.c_lflag = 0;
        toptions.c_cc[VMIN] = 0;      /* block untill n bytes are received */
        toptions.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */
        
        
        if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
            perror("init_serialport: Couldn't set term attributes");
            return -1;
        }
        
        return fd;
    }

    int serialport_close(int fd)
    {
        // 0 - ok
        // -1 - error
        return close(fd);
    }
#endif
