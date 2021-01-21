#ifndef _LOG_H_
#define _LOG_H_
#include "FileOperator.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h> 
#include <iostream> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sstream>
namespace stcontroller{
class Log{

    public:
        Log(std::vector<std::string> files);
        ~Log();

    public:
        void saveData(std::string name, const std::vector<float>& data);
        void step();
    private:
        std::vector<FileOperator *> filesOperator;     
        uint8_t files_num;
        uint16_t file_buffer_size;
        std::map<std::string,int> files_index;

        long unsigned int t;   

};
}




#endif
