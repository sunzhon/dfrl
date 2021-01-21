#include "plotData.h"

plotData::plotData(const char *fifoname,unsigned int size){
	path="/home/suntao/workspace/gorobots/controllers/stbot/genesis/study-tool/fifo";
	path += fifoname;
	pathname =path.c_str();
	char temp[100]="/home/suntao/workspace/gorobots/controllers/stbot/genesis/study-tool/studyPlot ";
	strcat(temp,fifoname);
	strcat(temp, " ");
	std::string temp1= std::to_string(size);
	strcat(temp,temp1.c_str());
	plotapp = temp;
	if((pid = fork())<0){
		perror("fork fail!");
		std::cout<<"fork fail!"<<std::endl;
		exit(1);
	}else if(pid ==0){
			childpid = getpid();
			std::cout<<"this is child process: "<<childpid<<std::endl;
			std::cout<<"plotapp is : "<<plotapp<<std::endl;
			execl("/bin/sh", "sh", "-c", plotapp, (char*)0);
			exit(127);
	}else{
		parentpid = getpid();
		std::cout<<"this is parent process: "<<parentpid<<std::endl;
		std::cout<<"fifo pathname is : "<<pathname<<std::endl;
		buf.resize(size);
		fifo = new ipcFifo(pathname);
		std::cout<<" create fifo ok"<<std::endl;
		fifo->openFifo(O_WRONLY);
		std::cout<<"open fifo ok"<<std::endl;
	}
	
}

plotData::~plotData(){
	delete fifo;
}
void plotData::setData(std::vector<double> data){
	for(unsigned int i=0;i<data.size();i++)
		buf.at(i) = data.at(i);
}
void plotData::update(){
	if(parentpid ==getpid()){
		fifo->setSendData(buf);
		fifo->send();
	}
}

void plotData::plot(std::vector<double> data){
	setData(data);
	update();
}


