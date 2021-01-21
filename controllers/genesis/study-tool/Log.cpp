#include "Log.h"
namespace stcontroller{
Log::Log(std::vector<std::string> files){
    //0) get current time
    struct timeval tv;
    struct timezone tz;
    struct tm *tm;
    string month="0";
    string day="0";
    string  hour="0";
    string minute="0";
    string second="0";
    gettimeofday(&tv,&tz);
    std::ostringstream ostr;
    tm=localtime(&tv.tv_sec);
    if(tm->tm_mon+1<10)
        ostr<<month<<tm->tm_mon + 1 ;
    else
        ostr << tm->tm_mon +1;

    if(tm->tm_mday<10)
        ostr<<day<<tm->tm_mday ;
    else
        ostr << tm->tm_mday;

    if(tm->tm_hour<10)
        ostr<<hour<<tm->tm_hour;
    else
        ostr << tm->tm_hour;

    if(tm->tm_min<10)
        ostr<<minute<<tm->tm_min;
    else
        ostr << tm->tm_min;

    if(tm->tm_sec<10)
        ostr<<second<<tm->tm_sec;
    else
        ostr << tm->tm_sec;
//
//    ostr << tm->tm_mday;
//    ostr << tm->tm_hour;
//    ostr << tm->tm_min;
//    ostr << tm->tm_sec;
    std::string folder = ostr.str();
    //1) init file path
    string parent_path = std::string(getenv("HOME")) + "/workspace/experiment_data/";
    string folder_path=parent_path+folder;
    //2) Creating a directory 
    if (mkdir(folder_path.c_str(), 0777) == -1) 
        cerr << "Error :  " << strerror(errno) << endl; 
    char mode[] = "w+";
    file_buffer_size = 40;
    files_num=files.size();
    filesOperator.resize(files_num);

    std::string file_path;
    for(int8_t idx=0;idx<files_num;idx++){
        file_path=folder_path +"/"+ files.at(idx) + ".csv";
        filesOperator.at(idx)=new FileOperator(file_path,mode,file_buffer_size);
        files_index.insert(pair<std::string, int>(files.at(idx),idx));  
    }
    t=1;
    //3) Information
    printf("##---------INFO---------##\n");
    printf("The date and time is :\n");
    printf("%d:%d:%d:%d:%02d:%02d %ld \n", tm->tm_year+1900,tm->tm_mon+1,tm->tm_mday, tm->tm_hour,tm->tm_min,tm->tm_sec, tv.tv_usec);
    printf("The data are stored at:\n");
    std::cout<<folder_path<<std::endl;
    printf("--------------------------\n");

    //4) store log which has the path of the data
     // 以写模式打开文件
   ofstream outfile;
   outfile.open(parent_path+"datapath.log",ios::in |ios::trunc);
   // 向文件写入用户输入的数据
   outfile << folder << endl;
   // 关闭打开的文件
   outfile.close();
   
}
Log::~Log(){
    for(uint8_t idx=0;idx<filesOperator.size();idx++)
        delete filesOperator.at(idx);
}
void Log::step(){
    t++;
    if(t>100000000)
        t=1;
    }
void  Log::saveData(std::string name, const std::vector<float>& data){
    std::map<std::string,int>::iterator files_itr;
    files_itr=files_index.find(name);
    uint8_t num=files_itr->second;
    assert(num<files_num);

    std::string strData = to_string(t);
    for(uint8_t idx=0;idx<data.size();idx++)
        strData +="\t" + std::to_string(data.at(idx));

    filesOperator[num]->AddData(t%file_buffer_size,strData);
    if(t%file_buffer_size == 0)
        filesOperator[num]->SaveToFile();
}
}
