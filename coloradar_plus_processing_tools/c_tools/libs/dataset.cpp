#include "dataset.h"

#include <fstream>
#include <sstream>


coloradar::ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToDataset) : datasetDirPath_(pathToDataset) {
    coloradar::internal::checkPathExists(datasetDirPath_);
    calibDirPath_ = datasetDirPath_ / "calib";
    coloradar::internal::checkPathExists(calibDirPath_);
    transformsDirPath_ = calibDirPath_ / "transforms";
    coloradar::internal::checkPathExists(transformsDirPath_);
    runsDirPath_ = datasetDirPath_ / "kitti";
    coloradar::internal::checkPathExists(runsDirPath_);

    imuTransform_ = loadTransform(transformsDirPath_ / "base_to_imu.txt");
    lidarTransform_ = loadTransform(transformsDirPath_ / "base_to_lidar.txt");
    initCascadeTransform();

    cascadeConfig_ = new coloradar::CascadeConfig(calibDirPath_);
}

Eigen::Affine3f coloradar::ColoradarPlusDataset::loadTransform(const std::filesystem::path& filePath) {
    coloradar::internal::checkPathExists(filePath);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;

    std::ifstream file(filePath);
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> translation.x() >> translation.y() >> translation.z();

    std::getline(file, line);
    iss.str(line);
    iss.clear();
    iss >> rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();

    transform.translate(translation);
    transform.rotate(rotation);
    return transform;
}

void coloradar::ColoradarPlusDataset::initCascadeTransform() {
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_radar.txt");
}

std::vector<std::string> coloradar::ColoradarPlusDataset::listRuns() {
    std::vector<std::string> runs;
    for (const auto& entry : std::filesystem::directory_iterator(runsDirPath_)) {
        if (entry.is_directory()) {
            runs.push_back(entry.path().filename().string());
        }
    }
    return runs;
}

coloradar::ColoradarPlusRun* coloradar::ColoradarPlusDataset::getRun(const std::string& runName) {
    return new coloradar::ColoradarPlusRun(runsDirPath_ / runName, cascadeConfig_);
}

std::vector<coloradar::ColoradarPlusRun*> coloradar::ColoradarPlusDataset::getRuns() {
    std::vector<std::string> runNames = listRuns();
    std::vector<coloradar::ColoradarPlusRun*> runs(runNames.size());
    for (size_t i = 0; i < runNames.size(); ++i)
        runs[i] = getRun(runNames[i]);
    return runs;
}

const Eigen::Affine3f& coloradar::ColoradarPlusDataset::imuTransform() const { return imuTransform_; }
const Eigen::Affine3f& coloradar::ColoradarPlusDataset::lidarTransform() const { return lidarTransform_; }
const Eigen::Affine3f& coloradar::ColoradarPlusDataset::cascadeTransform() const { return cascadeTransform_; }
const coloradar::RadarConfig* coloradar::ColoradarPlusDataset::cascadeConfig() const { return cascadeConfig_; }


coloradar::ColoradarDataset::ColoradarDataset(const std::filesystem::path& pathToDataset) : coloradar::ColoradarPlusDataset(pathToDataset) {
    singleChipTransform_ = loadTransform(transformsDirPath_ / "base_to_single_chip.txt");
    singleChipConfig_ = new coloradar::SingleChipConfig(calibDirPath_);
}

void coloradar::ColoradarDataset::initCascadeTransform() {
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_cascade.txt");
}

coloradar::ColoradarPlusRun* coloradar::ColoradarDataset::getRun(const std::string& runName) {
    return new coloradar::ColoradarRun(runsDirPath_ / runName, cascadeConfig_, singleChipConfig_);
}

const Eigen::Affine3f& coloradar::ColoradarDataset::singleChipTransform() const { return singleChipTransform_; }
const coloradar::RadarConfig* coloradar::ColoradarDataset::singleChipConfig() const { return singleChipConfig_; }
