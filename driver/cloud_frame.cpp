#include "cloud_frame.hpp"
#include <unistd.h>

frame::frame(const frame& f){
    this->calibration_file_n = f.calibration_file_n;
    this->dump_file_n = f.dump_file_n;
    this->matrix_ = f.matrix_;
    std::cout<<"frame copy construct"<<std::endl;
}
void frame::setup(){
    input_.close();
    input_.setup(dump_file_n);
    last_azimuth_ = 0.0f;
    raw_data_.setup(calibration_file_n, 0.1, 200);
    raw_data_.setParameters(0.1, 200, 180, 360);
}

bool frame::get_frame(velodyne_rawdata::vcloud& cloud){
    last_azimuth_ = 0.0f;
    for(;;){
        float azimuth;
        uint8_t pkt[pkt_data_len];
        input_.getPacket(pkt,1);
        raw_data_.unpack(pkt, matrix_, cloud, azimuth);
        // std::cout<<"azimuth: "<<azimuth<<std::endl;
        if(azimuth<last_azimuth_){
            // std::cout<<"break azimuth: "<<azimuth<<std::endl;
            return true;
        }
        last_azimuth_ = azimuth;
        // std::cout<<"azimuth: "<<azimuth<<std::endl;
    }
}

void frame::print_matrix(){
    std::cout<<"matrix for "<<dump_file_n<<": \n";
    std::cout<<"[ ";
    for(auto e: matrix_){
        std::cout<<e<<" ";
    }
    std::cout<<"]"<<std::endl;
}