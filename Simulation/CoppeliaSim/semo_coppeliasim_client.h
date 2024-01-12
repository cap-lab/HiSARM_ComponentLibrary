#ifndef __SEMO_COPPELIASIM_CLIENT_HEADER__
#define __SEMO_COPPELIASIM_CLIENT_HEADER__
#include "RemoteAPIClient.h"
#include "semo_logger.h"
#include "semo_common.h"

extern bool started;

class RemoteAPIClientWrapper {
private:
    RemoteAPIClient *client;
    std::mutex client_mtx;
    int robot_id;

    json call(std::string command, json args) {
         std::lock_guard<std::mutex> lock(client_mtx);
        return client->call(command, args);
    }
public:
    RemoteAPIClientWrapper(std::string ip, int port, int robot_id) {
        this->robot_id = robot_id;
        client = new RemoteAPIClient(ip, port, -1, -1);
        start();
    }
    RemoteAPIClientWrapper(const RemoteAPIClientWrapper&) = delete;
    RemoteAPIClientWrapper& operator=(const RemoteAPIClientWrapper&) = delete;

    void start();
    int64_t get_object(std::string path, std::optional<json> options);
    int64_t get_parent_object(int64_t objectHandle);
    void set_joint_target_velocity(int64_t objectHandle, double targetVelocity);
    void set_led(int64_t lightHandle, int64_t state, double *value);
    void get_position(int64_t objectHandle, int64_t relativeObjectHandle, double *buffer);
    void get_orientation(int64_t objectHandle, int64_t relativeObjectHandle, double *buffer);
    int64_t get_proximity(int64_t sensorHandle, double *dist);
    int32_t get_int32_signal(const char* signalName, int32_t *buffer);
    void set_int32_signal(const char* signalName, int64_t value);
    void set_remove_object(int64_t objectHandle);
    void set_remove_objects(std::vector<int64_t> objectHandles);
    std::vector<int64_t> get_objects_in_tree(int64_t baseObject);
    std::vector<uint8_t> get_vision_sensor_img(int64_t sensorHandle);
    void get_function(std::string command, json _args, int64_t size, void *buffer);
    void set_function(std::string command, json _args);
};

#endif