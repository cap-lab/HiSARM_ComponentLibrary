#include "turtlebot3_lds.h"

static LiPkg *pkg = NULL;
static int init = 0;
static CmdInterfaceLinux cmd_port(8); //lds_version = 8

void init_lds(){
    if (init == 0){
        pkg = new LD08_LiPkg;
        init = 1;

        std::vector<std::pair<std::string, std::string>> device_list;
        std::string port_name;
        cmd_port.GetCmdDevices(device_list);
        for (auto n : device_list){
            // std::cout << n.first << "    " << n.second << std::endl;
            if (strstr(n.second.c_str(), "CP2102")){
            port_name = n.first;
            }
        }
    
        if (port_name.empty() == false){
            // std::cout << "FOUND LiDAR" << std::endl;
            cmd_port.SetReadCallback([&pkg](const char *byte, size_t len){
                if(pkg->Parse((uint8_t*)byte, len)){
                    pkg->AssemblePacket();  
                } 
            });
        }
    
        if (cmd_port.Open(port_name)){
            // std::cout << "LiDAR started successfully " << std::endl;
        }
    }
}

bool isFrameReady() {
    return pkg->IsFrameReady();
}

FrameData getFrameData() {
    return pkg->GetFrameData();
}

int getFrontDistance(const FrameData &data, int l_angle, int r_angle){
    float angle_increment = (data.angle_max - data.angle_min) / data.len;
    float angle_point = data.angle_min; //1st point's angle

    int front_distance=8000; //max_distance = 8m, distance = {front, left, back, right}
    for (int i = 0; i < (int)data.len; i++){
        // check one of 10 points because there are too many points.
        angle_point += angle_increment;
        if(data.intensities[i]>100){ //if data is reliable
            if(angle_point < l_angle || r_angle < angle_point){ //front
                front_distance = (front_distance < data.distance[i]) ? front_distance : data.distance[i];
            }
        }
    }

    return front_distance;
}