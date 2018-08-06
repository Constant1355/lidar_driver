#pragma once

#include <stdint.h>
#include <iostream>
#include <fstream>

namespace ad {
namespace lidar {

#define pkt_len 1212
#define pkt_data_len 1200
#define time_len 12

class InputDumpFile
{
public:
    InputDumpFile(){};
    ~InputDumpFile();
    void setup(std::string fileN);
    void close();
    int getPacket(uint8_t* pkt, const int& direct);
    bool is_open;
    struct file_info{
        size_t pkt_number;
        __uint64_t start_s;
        __uint64_t end_s;
    } info;

private:
    std::string file_name_;
    std::streampos start_read_pos_;
    std::streampos end_read_pos_;
    std::ifstream fin_;
};

} // namespace lidar
} // namespace ad
