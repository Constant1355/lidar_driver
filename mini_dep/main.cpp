#include <iostream>
#include <array>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <unistd.h>

#include "../driver/cloud_frame.hpp"
#include "../GL_displayer/displayer.hpp"

template<typename T> void operator >>(const YAML::Node& node, T& v){
      v = node.as<T>();
    }

extern velodyne_rawdata::vcloud disp_cloud;
extern std::mutex disp_mtx;

void input_matrixs(matrixs& ms);
void print_matrixs(const matrixs& ms);

int main(int argc, char* argv[]){
    matrixs ms;
    input_matrixs(ms);
    print_matrixs(ms);
    std::vector<frame> frames;
    std::cout<<"argc: "<<argc<<std::endl;
    if(argc >=2){
        for(size_t idx=0; idx < argc-1; ++idx){
            frames.push_back(
                frame(argv[idx+1],"../VLP16db.yaml",ms[idx]));
            std::cout<<"push: "<<idx<<std::endl;
        }
        
    }else{
        frames.push_back(
            frame("../lidar_forward","../VLP16db.yaml",ms[0]));
    }
    std::cout<<"befor setuup"<<std::endl;
    for(auto& f: frames){
        f.setup();
        f.print_matrix();
    }
    std::thread gl_show(GL_show, argc, argv);
    for(;;){
        velodyne_rawdata::vcloud cloud;
        disp_mtx.lock();
        disp_cloud.clear();
        for(auto& f: frames){
            f.get_frame(disp_cloud);
        }
        disp_mtx.unlock();
        usleep(10000);
        // std::cout<<"disp_cloud.size: "<<disp_cloud.size()<<std::endl;
    }
    // GL_show(argc, argv);
    return 1;
}

void input_matrixs(matrixs& m){
    YAML::Node conf = YAML::LoadFile("../config.yaml");
    YAML::Node m_node = conf["matrix"];
    for(auto n: m_node){
        if(n.size()==16){
            matrix temp_m;
            for(size_t idx = 0; idx<16; ++idx){
                temp_m[idx] = n[idx].as<float>();
            }
            m.push_back(temp_m);
        }else{
            std::cout<<"matrix not exis: "<<std::endl;
        }
    }
}
void print_matrixs(const matrixs& ms){
    std::cout<<"there are "<<ms.size()<<" matrixs"<<std::endl;
    for(auto m: ms){
        std::cout<<"[ ";
        for(auto e: m){
            std::cout<<e<<" ";
        }
        std::cout<<"]"<<std::endl;
    }
}