#include "ipcFifo.h"
/*
 * *
 * Author: SUN TAO
 * Date:2018
 * Description:
 * ipcFifo is a class to use FIFO to transfer data,
 * */

ipcFifo::ipcFifo(const char * path){
	fifoPath = path;
	if((access(fifoPath,F_OK))== -1){//wenjian bu chun zai
		if(mkfifo(fifoPath,0666)<0 &&errno != EEXIST)
			perror("creat FIFO fail!");	
	}
	memset(sendbuf,0.0,BUFSIZE);
	memset(receivebuf,0.0,BUFSIZE);
}
ipcFifo::~ipcFifo(){
	close(fd);
	unlink(fifoPath);
}
void ipcFifo::openFifo(int mode){
	if(mode == O_RDONLY)
		mode =mode|O_NONBLOCK;
	if((fd =open(fifoPath, mode)) < 0){
		perror("open FIFO fail");
		exit(1);
	}
}

void ipcFifo::setSendData( std::vector<double> &buf){
	for(unsigned int i=0;i < buf.size();i++)
		sendbuf[i]=buf.at(i);
}
void ipcFifo::setSendData( double buf){
		sendbuf[0]= buf;
}


void ipcFifo::getReceiveData( std::vector<double> &buf){
	for(unsigned int i=0;i < buf.size();i++)
		buf.at(i)=receivebuf[i];
}

int ipcFifo::send(){
   int len = 0;
	if((len = write(fd,sendbuf,sizeof(sendbuf)))< 0){
		perror("Write FIFO Failed");
		std::cout<<"Write FIFO Faild"<<std::endl;
		close(fd);
		exit(1);
	}
	return len;
}

int ipcFifo::receive(){
	int len =0;
	len=read(fd,receivebuf,sizeof(receivebuf));
	return len;
}

