#include "ofxSerial.h"

ofxSerial::ofxSerial(std::string sPort, int nBaud)
:port(sPort)
,baud(nBaud)
{
	std::cout << port << std::endl;
	fd = openPort(port.c_str(), baud);
	if(fd == -1) {
		std::cout << "ERROR: could not connect to serial port: " << sPort << std::endl;
	}
}

bool ofxSerial::readUntil(std::string& rResult, char cUntil) {
	char b[1];
	char buf[1];
	int  i = 0;

	do { 
		int n = read(fd, b, 1);  // read a char at a time
		if( n == -1) return false;    // couldn't read
		if( n == 0 ) {
			return false;
		}
		buffer.push_back(b[0]);
		i++;
	} while( b[0] != cUntil );
	
	std::vector<char>::iterator it = buffer.begin();
	while(it != buffer.end()) {
		rResult.push_back((*it));
		++it;
	}
	buffer.clear();
	return true;

}

int ofxSerial::writeBytes(const char* aData, int nNumBytes) {
	if(fd == -1) {
		std::cout << "ERROR: no open port" << std::endl;
		return -1;
	}
	int n = write(fd, aData, nNumBytes);
    if( n!=nNumBytes) 
        return -1;
    return n;
}
												


// Open a port.
// ------------------------------------------------------
int ofxSerial::openPort(const char* sPort, int nBaud) {
	struct termios toptions;
	int fd;
	
	fd = open(sPort, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1)  {
		perror("init_serialport: Unable to open port ");
		return -1;
	}
	
	if (tcgetattr(fd, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes");
		return -1;
	}
	speed_t brate = nBaud; // let you override switch below if needed
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
	
	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;
	
	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	
	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw
	
	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 20;
	
	if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
		return -1;
	}
	return fd;
}
