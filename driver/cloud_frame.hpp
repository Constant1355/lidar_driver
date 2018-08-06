#pragma once

#include "input_dump_file.hpp"
#include <array>
#include <vector>
#include "rawdata.h"
typedef std::array<float,16> matrix;
typedef std::vector<matrix> matrixs;

const matrix identity_matrix ={
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
};

class frame{
    public:
    frame();
    frame(std::string d_file_n, std::string c_file_n)
        :dump_file_n(d_file_n),calibration_file_n(c_file_n),
        matrix_(identity_matrix)
        {};
    frame(std::string d_file_n, std::string c_file_n, matrix m)
        :frame(d_file_n,c_file_n)
        {matrix_ = m;};

    frame(const frame& f);
    void setup();
    bool get_frame(velodyne_rawdata::vcloud& cloud);
    void print_matrix();

    private:
    std::string dump_file_n;
    std::string calibration_file_n;

    velodyne_rawdata::RawData raw_data_;
    ad::lidar::InputDumpFile input_;
    float last_azimuth_;
    std::array<float,16> matrix_;
};