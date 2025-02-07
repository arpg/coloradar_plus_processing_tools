#ifndef DEVICE_H
#define DEVICE_H

#include "utils.h"


namespace coloradar {

enum class DeviceType {
    Lidar,
    CascadeRadar,
    SingleChipRadar,
    Imu,
    Base
};

class Device {
protected:
    DeviceType type_;
    std::string name_;

    static std::string deviceToName(DeviceType device) {
        switch (device) {
            case DeviceType::Lidar: return "lidar";
            case DeviceType::CascadeRadar: return "cascade_radar";
            case DeviceType::SingleChipRadar: return "single_chip_radar";
            case DeviceType::Imu: return "imu";
            case DeviceType::Base: return "base";
            default: throw std::invalid_argument("Unknown device type");
        }
    }

    static DeviceType nameToDevice(const std::string& name) {
        static const std::unordered_map<std::string, DeviceType> deviceMap = {
            {"lidar", DeviceType::Lidar},
            {"cascade_radar", DeviceType::CascadeRadar},
            {"single_chip_radar", DeviceType::SingleChipRadar},
            {"imu", DeviceType::Imu},
            {"base", DeviceType::Base}
        };
        auto it = deviceMap.find(name);
        if (it == deviceMap.end()) {
            throw std::invalid_argument("Unknown device: " + name);
        }
        return it->second;
    }

public:
    Device() : type_(DeviceType::Base), name_("base") {}
    Device(DeviceType type) : type_(type), name_(deviceToName(type)) {}
    Device(std::string name) : type_(nameToDevice(name)), name_(name) {}

    DeviceType type() const { return type_; }
    std::string name() const { return name_; }
};

}