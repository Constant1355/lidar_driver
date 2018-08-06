#include "input_dump_file.hpp"

namespace ad{
namespace lidar{

InputDumpFile::~InputDumpFile(){
    fin_.close();
}

void InputDumpFile::close(){
    fin_.close();
    is_open = false;
}

void InputDumpFile::setup(std::string fileN){
    file_name_ = fileN;
    fin_.open(file_name_.c_str(), std::ios_base::binary);
    if (!fin_.is_open())
    {
        is_open = false;
        std::cout << "Error Out Open file name: " << fileN<< std::endl;
        return;
    }
    is_open = true;

    char begin[time_len], end[time_len];
    size_t tail = 0;
    fin_.seekg(0,std::ios::end);
    tail = fin_.tellg() % pkt_len;
    info.pkt_number = fin_.tellg()/pkt_len;

    fin_.seekg(pkt_data_len, std::ios::beg);
    fin_.read(begin, sizeof(begin));
    fin_.seekg(-sizeof(end)-tail, std::ios::end);
    fin_.read(end, sizeof(end));
    
    info.start_s = *(__uint64_t*)begin;
    info.end_s = *(__uint64_t*)end;

    std::cout<<fileN<<"\n--info: \n";
    std::cout<<" --pkt_number: "<<info.pkt_number<<std::endl;
    std::cout<<" --   start_s: "<<info.start_s<<std::endl;
    std::cout<<" --   end_s: "<<info.end_s<<std::endl;

    start_read_pos_ =0;
    end_read_pos_ = info.pkt_number*pkt_len;
    fin_.seekg(start_read_pos_,std::ios::beg);
}
int InputDumpFile::getPacket(uint8_t* pkt, const int& direct){
    char timeStamp[12];
    __uint64_t current_pkt_s;
    __uint32_t current_pkt_us;
    if(direct<0){
        //move seekg backward two pkts
        std::streampos current_pos = fin_.tellg();
        std::streampos frame_len = 1212;
        if(current_pos < 2*frame_len){
            fin_.seekg(-frame_len, std::ios::end);
        }else{
            fin_.seekg(-2*frame_len, std::ios::cur);
        }
    }
    // read forward

    if(fin_.tellg() >= end_read_pos_){
        fin_.seekg(start_read_pos_,std::ios::beg);
    }
    fin_.read((char *)pkt, pkt_data_len);
    fin_.read(timeStamp, sizeof(timeStamp));
    current_pkt_s = *(__uint64_t*)timeStamp;
    current_pkt_us = *(__uint32_t*)(timeStamp+8);
    return 0;

}

} // namespace lidar
} // namespace ad
