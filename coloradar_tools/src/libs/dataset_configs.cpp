#include "dataset_configs.h"

#include <chrono>
#include <ctime>


namespace coloradar {

YAML::Node DatasetExportConfig::findNode(const YAML::Node &config, const std::string &nestedKey) {
    std::istringstream iss(nestedKey);
    std::string token;
    YAML::Node node = config;
    while (std::getline(iss, token, '.')) {
        if (!node[token])
            return YAML::Node();
        node = node[token];
    }
    return node;
}

bool DatasetExportConfig::parseBoolKey(const YAML::Node &config, const std::string &nestedKey, bool defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    try {
        return node.as<bool>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed bool value for key: " + nestedKey);
    }
}

int DatasetExportConfig::parseIntKey(const YAML::Node &config, const std::string &nestedKey, int defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    try {
        return node.as<int>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed int value for key: " + nestedKey);
    }
}

float DatasetExportConfig::parseFloatKey(const YAML::Node &config, const std::string &nestedKey, float defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    try {
        return node.as<float>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed float value for key: " + nestedKey);
    }
}

std::string DatasetExportConfig::parseDeviceName(const YAML::Node &config, const std::string &nestedKey, std::string defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    return node.as<std::string>();
}

std::filesystem::path DatasetExportConfig::parseDestination(const YAML::Node &config, const std::filesystem::path &defaultDestination) {
    YAML::Node node = findNode(config, "global.destination");
    if (!node.IsDefined() || node.IsNull()) {
        return defaultDestination;
    }
    std::filesystem::path destination = node.as<std::string>();
    if (destination.empty()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S.h5", std::localtime(&now_c));
        destination = "dataset_" + std::string(buffer);
    }
    if (destination.extension() != ".h5") {
        destination += ".h5";
    }
    validateDestination(destination);
    return destination;
}

std::vector<std::string> DatasetExportConfig::parseRuns(const YAML::Node &config) {
    YAML::Node node = findNode(config, "global.runs");
    std::vector<std::string> runs = {};
    if (!node.IsDefined() || node.IsNull()) {
        return runs;
    }
    if (node.IsSequence()) {
        for (const auto &n : node) {
            runs.push_back(n.as<std::string>());
        }
    } else {
        runs.push_back(node.as<std::string>());
    }
    validateRuns(runs);
    return runs;
}

void DatasetExportConfig::validateDestination(const std::filesystem::path &destination) {
    if (!std::filesystem::exists(destination.parent_path())) {
        std::filesystem::create_directories(destination.parent_path());
    }
    if (destination.extension() != ".h5") {
        throw std::runtime_error("Invalid file extension for destination: " + destination.string());
    }
}

void DatasetExportConfig::validateRuns(const std::vector<std::string> &runs) {
    for (const auto &run : runs) {
        if (run.empty() || run == "all") {
            continue;
        }
    }
}

void DatasetExportConfig::validateDeviceName(const std::string &name) {
    if (devices_.find(name) == devices_.end()) {
        throw std::runtime_error("Unknown device: " + name);
    }
}

DatasetExportConfig::DatasetExportConfig(const std::string &yamlFilePath) {
    YAML::Node config = YAML::LoadFile(yamlFilePath);
    destinationFilePath_ = parseDestination(config, destinationFilePath_);
    runs_ = parseRuns(config);
    exportTransforms_ = parseBoolKey(config, "global.export_transforms", exportTransforms_);

    cascade_.exportPoses = parseBoolKey(config, "devices.cascade.export_poses", cascade_.exportPoses);
    cascade_.exportTimestamps = parseBoolKey(config, "devices.cascade.export_timestamps", cascade_.exportTimestamps);

    lidar_.exportPoses = parseBoolKey(config, "devices.lidar.export_poses", lidar_.exportPoses);
    lidar_.exportTimestamps = parseBoolKey(config, "devices.lidar.export_timestamps", lidar_.exportTimestamps);
    lidar_.collapseElevation = parseBoolKey(config, "devices.lidar.collapse_elevation", lidar_.collapseElevation);
    lidar_.collapseElevationMinZ = parseFloatKey(config, "devices.lidar.collapse_elevation_min_z_meters", lidar_.collapseElevationMinZ);
    lidar_.collapseElevationMaxZ = parseFloatKey(config, "devices.lidar.collapse_elevation_max_z_meters", lidar_.collapseElevationMaxZ);

    base_.exportPoses = parseBoolKey(config, "devices.base.export_poses", base_.exportPoses);
    base_.exportTimestamps = parseBoolKey(config, "devices.base.export_timestamps", base_.exportTimestamps);

    imu_.exportPoses = parseBoolKey(config, "devices.imu.export_poses", imu_.exportPoses);
    imu_.exportTimestamps = parseBoolKey(config, "devices.imu.export_timestamps", imu_.exportTimestamps);
    imu_.exportData = parseBoolKey(config, "devices.imu.export_data", imu_.exportData);

    singleChip_.exportPoses = parseBoolKey(config, "devices.single_chip.export_poses", singleChip_.exportPoses);
    singleChip_.exportTimestamps = parseBoolKey(config, "devices.single_chip.export_timestamps", singleChip_.exportTimestamps);
}

DatasetExportConfig::DatasetExportConfig(
    const std::filesystem::path &destinationFilePath,
    const std::vector<std::string> &runs,
    bool exportTransforms,
    const RadarExportConfig &cascade,
    const LidarExportConfig &lidar,
    const BaseExportConfig &base,
    const ImuExportConfig &imu,
    const RadarExportConfig &singleChip
)
    : destinationFilePath_(destinationFilePath),
      runs_(runs),
      exportTransforms_(exportTransforms),
      cascade_(cascade),
      lidar_(lidar),
      base_(base),
      imu_(imu),
      singleChip_(singleChip)
{
    validateDestination(destinationFilePath_);
    validateRuns(runs_);
    validateDeviceName(lidar_.centerSensor);
}

const std::filesystem::path &DatasetExportConfig::destinationFilePath() const {
    return destinationFilePath_;
}

const std::vector<std::string> &DatasetExportConfig::runs() const {
    return runs_;
}

bool DatasetExportConfig::exportTransforms() const {
    return exportTransforms_;
}

const RadarExportConfig &DatasetExportConfig::cascade() const {
    return cascade_;
}

const LidarExportConfig &DatasetExportConfig::lidar() const {
    return lidar_;
}

const BaseExportConfig &DatasetExportConfig::base() const {
    return base_;
}

const ImuExportConfig &DatasetExportConfig::imu() const {
    return imu_;
}

const RadarExportConfig &DatasetExportConfig::singleChip() const {
    return singleChip_;
}

}
