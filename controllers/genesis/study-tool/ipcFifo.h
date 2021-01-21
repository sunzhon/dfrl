#ifndef IPCFIFO_H_
#define IPCFIFO_H_

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <iostream>
#define FIFOPATH "./fifo1"
#define BUFSIZE 1024
class ipcFifo{
public:
	ipcFifo(const char * path);
	virtual ~ipcFifo();
	void openFifo(int mode);
	void setSendData(std::vector<double> &buf);
	void setSendData(double buf);
	void getReceiveData(std::vector<double> &buf);
	int send();
	int receive();
private:
	int fd;
	const char * fifoPath;
	double sendbuf[BUFSIZE];
	double receivebuf[BUFSIZE];
};


#endif
