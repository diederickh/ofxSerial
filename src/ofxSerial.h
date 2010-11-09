#ifndef OFXSERIALH
#define OFXSERIALH

// For rs232 on MAC
// ---------------------------------------------------------
#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File controdl definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
// ---------------------------------------------------------

#include <string>
#include <vector>
#include <iostream>

class ofxSerial {
public:
	
	ofxSerial(std::string sPort, int nBaud);
	bool readUntil(std::string& rResult, char cUntil);
	int writeBytes(const char* aData, int nNumBytes);

private:
	int openPort(const char* sPort, int nBaud);
	
	int fd;
	std::vector<char> buffer;
	std::string port;
	int baud;
};
#endif