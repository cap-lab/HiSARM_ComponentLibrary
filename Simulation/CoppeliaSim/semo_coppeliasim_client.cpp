#include "semo_coppeliasim_client.h"

using namespace std;

bool started = false;

void RemoteAPIClientWrapper::start() {
    client->step(false);
    if (started == false){
        started = true;
        json _args(json_array_arg);
        this->call("sim.startSimulation", _args);
    }
}

int64_t RemoteAPIClientWrapper::get_object(string path, optional<json> options) {
    try {
        json _args(json_array_arg);
        _args.push_back(path);
        if(options)
        {
            _args.push_back(*options);
        }
        auto _ret = call("sim.getObject", _args);
        return _ret[0].as<int64_t>();
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s, object: %s", e.what(), path.c_str());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
        return -1;
    }
}


int64_t RemoteAPIClientWrapper::get_parent_object(int64_t objectHandle) {
    try {
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = call("sim.getObjectParent", _args);
        return _ret[0].as<int64_t>();
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
        return -1;
    }
}

void RemoteAPIClientWrapper::set_joint_target_velocity(int64_t objectHandle, double targetVelocity) {
    try {
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(targetVelocity);
        auto _ret = call("sim.setJointTargetVelocity", _args);
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

void RemoteAPIClientWrapper::set_led(int64_t lightHandle, int64_t state, double *value) {
    try {
        json _args(json_array_arg);
        _args.push_back(lightHandle);
        _args.push_back(state);
        _args.push_back(nullptr);
        std::vector<double> params;
        params.push_back(value[0]);
        params.push_back(value[1]);
        params.push_back(value[2]);
        _args.push_back(params);
        _args.push_back(params);
        call("sim.setLightParameters", _args);
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

void RemoteAPIClientWrapper::get_position(int64_t objectHandle, int64_t relativeObjectHandle, double *buffer) {
    try {
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeObjectHandle);
        auto _ret = call("sim.getObjectPosition", _args);
        auto _vec = _ret[0].as<std::vector<double>>();
        buffer[0] = _vec.at(0);
        buffer[1] = _vec.at(1);
        buffer[2] = _vec.at(2);
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

void RemoteAPIClientWrapper::get_orientation(int64_t objectHandle, int64_t relativeObjectHandle, double *buffer) {
    try {
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        _args.push_back(relativeObjectHandle);
        auto _ret = call("sim.getObjectOrientation", _args);
        auto _vec = _ret[0].as<std::vector<double>>();
        buffer[0] = _vec.at(0);
        buffer[1] = _vec.at(1);
        buffer[2] = _vec.at(2);
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

int64_t RemoteAPIClientWrapper::get_proximity(int64_t sensorHandle, double *dist) {
    try {
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = call("sim.readProximitySensor", _args);
        *dist = _ret[1].as<double>();
        return _ret[0].as<int64_t>();
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
        return -1;
    }
}

int32_t RemoteAPIClientWrapper::get_int32_signal(const char* signalName, int32_t *buffer) {
    try {
        json _args(json_array_arg);
        _args.push_back(std::string(signalName));
        auto _ret = call("sim.getInt32Signal", _args);
        if (!_ret.empty()) {
            *buffer = _ret[0].as<int32_t>();
            return TRUE;
        } else {
            return FALSE;
        }
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
        return FALSE;
    }
}

void RemoteAPIClientWrapper::set_int32_signal(const char* signalName, int64_t value) {
    try {
        json _args(json_array_arg);
        _args.push_back(std::string(signalName));
        if (value < 0){
            call("sim.clearInt32Signal", _args);
        } else {
            _args.push_back(value);
            client->call("sim.setInt32Signal", _args);
        }
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

void RemoteAPIClientWrapper::set_remove_object(int64_t objectHandle) {
    try {
        json _args(json_array_arg);
        _args.push_back(objectHandle);
        auto _ret = call("sim.removeObject", _args); 
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

void RemoteAPIClientWrapper::set_remove_objects(std::vector<int64_t> objectHandles) {
    try {
        json _args(json_array_arg);
        _args.push_back(objectHandles);
        auto _ret = call("sim.removeObjects", _args); 
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

std::vector<int64_t> RemoteAPIClientWrapper::get_objects_in_tree(int64_t baseObject) {
    try {
        json _args(json_array_arg);
        _args.push_back(baseObject);
        auto _ret = call("sim.getObjectsInTree", _args);
        return _ret[0].as<std::vector<int64_t>>();
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
        return std::vector<int64_t>();
    }
}

std::vector<uint8_t> RemoteAPIClientWrapper::get_vision_sensor_img(int64_t sensorHandle) {
    try {
        json _args(json_array_arg);
        _args.push_back(sensorHandle);
        auto _ret = call("sim.getVisionSensorImg", _args);
        if (_ret[0].is_byte_string()) {
            return _ret[0].as<std::vector<uint8_t>>();
        } else {
            return std::vector<uint8_t>();
        }
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
        return std::vector<uint8_t>();
    }
}

void RemoteAPIClientWrapper::get_function(std::string command, json _args, int64_t size, void *buffer) {
    try {
        auto _ret = call(command, _args);
    // memset(buffer, _ret.data, size);
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}

void RemoteAPIClientWrapper::set_function(std::string command, json _args) {
    try {
        call(command, _args);
    } catch (const zmq::error_t &e) {
        SEMO_LOG_ERROR("simmulation error: %s", e.what());
        std::cerr << "ZMQ Error: " << e.what() << std::endl;
    }
}