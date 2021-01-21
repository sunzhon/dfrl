#ifndef PLOTDATA_H_
#define PLOTDATA_H_

#include "ipcFifo.h"
#include <time.h>
#include <math.h>
#include <string>
#include <iostream>
class plotData{
	public:
		plotData(const char *fifoname,unsigned int size);// size is the line neumber that you want to plot
		~plotData();
		void plot(std::vector<double> data);
	private:
		std::string path;
		const char * pathname;
		const char * plotapp;
		std::vector<double> buf;
		ipcFifo *fifo;
		pid_t pid;
		pid_t parentpid;
		pid_t childpid;
	private:
		void update();
		void setData(std::vector<double> data);//size is the data number to plot
};


#endif
