#include "coloradar_run.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>


coloradar::ColoradarPlusRun::ColoradarPlusRun(const std::filesystem::path& runPath, coloradar::RadarConfig* cascadeRadarConfig) : runDirPath_(runPath), name(runDirPath_.filename()), cascadeConfig_(cascadeRadarConfig) {
    coloradar::internal::checkPathExists(runDirPath_);
    posesDirPath_ = runDirPath_ / "groundtruth";
    coloradar::internal::checkPathExists(posesDirPath_);
    imuDirPath_ = runDirPath_ / "imu";
    coloradar::internal::checkPathExists(imuDirPath_);
    lidarScansDirPath_ = runDirPath_ / "lidar";
    coloradar::internal::checkPathExists(lidarScansDirPath_);
    cascadeScansDirPath_ = runDirPath_ / "cascade";
    coloradar::internal::checkPathExists(cascadeScansDirPath_);

    lidarCloudsDirPath_ = lidarScansDirPath_ / "pointclouds";
    coloradar::internal::checkPathExists(lidarCloudsDirPath_);
    lidarMapsDirPath_ = runDirPath_ / "lidar_maps";

    cascadeCubesDirPath_ = cascadeScansDirPath_ / "adc_samples";
    coloradar::internal::checkPathExists(cascadeCubesDirPath_);
    cascadeHeatmapsDirPath_ = cascadeScansDirPath_ / "heatmaps";
    coloradar::internal::checkPathExists(cascadeHeatmapsDirPath_);
    cascadeCloudsDirPath_ = cascadeScansDirPath_ / "pointclouds";

    poseTimestamps_ = readTimestamps(posesDirPath_ / "timestamps.txt");
    imuTimestamps_ = readTimestamps(imuDirPath_ / "timestamps.txt");
    lidarTimestamps_ = readTimestamps(lidarScansDirPath_ / "timestamps.txt");
    cascadeCubeTimestamps_ = readTimestamps(cascadeCubesDirPath_ / "timestamps.txt");
    cascadeTimestamps_ = readTimestamps(cascadeHeatmapsDirPath_ / "timestamps.txt");
}

std::vector<double> coloradar::ColoradarPlusRun::readTimestamps(const std::filesystem::path& path) {
    coloradar::internal::checkPathExists(path);
    std::vector<double> timestamps;
    std::ifstream infile(path);
    std::string line;
    while (std::getline(infile, line)) {
        timestamps.push_back(std::stod(line));
    }
    return timestamps;
}

const std::vector<double>& coloradar::ColoradarPlusRun::poseTimestamps() const { return poseTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::imuTimestamps() const { return imuTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::lidarTimestamps() const { return lidarTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::cascadeCubeTimestamps() const { return cascadeCubeTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::cascadeTimestamps() const { return cascadeTimestamps_; }

std::vector<int16_t> coloradar::ColoradarPlusRun::getDatacube(const std::filesystem::path& binFilePath, coloradar::RadarConfig* config) {
    coloradar::internal::checkPathExists(binFilePath);
    std::ifstream file(binFilePath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + binFilePath.string());
    }
    int totalElements = config->numTxAntennas * config->numRxAntennas * config->numChirpsPerFrame * config->numAdcSamplesPerChirp * 2;
    std::vector<int16_t> frameBytes(totalElements);
    file.read(reinterpret_cast<char*>(frameBytes.data()), totalElements * sizeof(int16_t));
    if (file.gcount() != totalElements * sizeof(int16_t)) {
        throw std::runtime_error("Datacube file read error or size mismatch");
    }
    file.close();
    return frameBytes;
}

std::vector<float> coloradar::ColoradarPlusRun::getHeatmap(const std::filesystem::path& binFilePath, coloradar::RadarConfig* config) {
    coloradar::internal::checkPathExists(binFilePath);
    std::ifstream file(binFilePath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + binFilePath.string());
    }
    int totalElements = config->numElevationBins * config->numAzimuthBins * config->numPosRangeBins * 2;
    std::vector<float> heatmap(totalElements);
    file.read(reinterpret_cast<char*>(heatmap.data()), totalElements * sizeof(float));
    if (file.gcount() != totalElements * sizeof(float)) {
        throw std::runtime_error("Heatmap file read error or size mismatch");
    }
    file.close();
    return heatmap;
}

void coloradar::ColoradarPlusRun::createRadarPointclouds(coloradar::RadarConfig* config, const std::filesystem::path& heatmapDirPath, const std::filesystem::path& pointcloudDirPath, const float& intensityThresholdPercent) {
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath);
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath / "data");
    for (auto const& entry : std::filesystem::directory_iterator(heatmapDirPath / "data")) {
        if (!entry.is_directory() && entry.path().extension() == ".bin") {
            std::filesystem::path heatmapPath = entry.path();
            std::vector<float> heatmap = getHeatmap(heatmapPath, config);
            pcl::PointCloud<coloradar::RadarPoint> cloud = coloradar::heatmapToPointcloud(heatmap, config, intensityThresholdPercent);

            std::string filename = heatmapPath.filename();
            filename.replace(filename.find("heatmap"), 7, "radar_pointcloud");
            std::filesystem::path cloudPath = pointcloudDirPath / "data" / filename;
            std::ofstream file(cloudPath, std::ios::out | std::ios::binary);
            if (!file.is_open()) {
                throw std::runtime_error("Failed to open file: " + cloudPath.string());
            }
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                file.write(reinterpret_cast<const char*>(&cloud.points[i].x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].z), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].intensity), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].doppler), sizeof(float));
            }
            file.close();
        }
    }
}

pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarPlusRun::getRadarPointcloud(const std::filesystem::path& binFilePath, RadarConfig* config, const float& intensityThresholdPercent) {
    pcl::PointCloud<coloradar::RadarPoint> cloud;
    std::ifstream file(binFilePath, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + binFilePath.string());
    }
    float maxIntensity = 0.0f;
    std::vector<coloradar::RadarPoint> points;

    while (file.good()) {
        coloradar::RadarPoint point;
        file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.doppler), sizeof(float));

        if (!file.good()) break;
        points.push_back(point);
        if (point.intensity > maxIntensity) {
            maxIntensity = point.intensity;
        }
    }
    file.close();

    float intensityThreshold = maxIntensity * (intensityThresholdPercent / 100.0f);
    for (const auto& point : points) {
        if (point.intensity >= intensityThreshold) {
            cloud.points.push_back(point);
        }
    }
    return cloud;
}


std::vector<int16_t> coloradar::ColoradarPlusRun::getCascadeDatacube(const std::filesystem::path& binFilePath) {
    return getDatacube(binFilePath, cascadeConfig_);
}
std::vector<int16_t> coloradar::ColoradarPlusRun::getCascadeDatacube(const int& cubeIdx) {
    return getCascadeDatacube(cascadeCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}
std::vector<float> coloradar::ColoradarPlusRun::getCascadeHeatmap(const std::filesystem::path& binFilePath) {
    return getHeatmap(binFilePath, cascadeConfig_);
}
std::vector<float> coloradar::ColoradarPlusRun::getCascadeHeatmap(const int& hmIdx) {
    return getCascadeHeatmap(cascadeHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}
void coloradar::ColoradarPlusRun::createCascadePointclouds(const float& intensityThresholdPercent) {
    createRadarPointclouds(cascadeConfig_, cascadeHeatmapsDirPath_, cascadeCloudsDirPath_, intensityThresholdPercent);
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarPlusRun::getCascadePointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent) {
    return getRadarPointcloud(binFilePath, cascadeConfig_, intensityThresholdPercent);
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarPlusRun::getCascadePointcloud(const int& cloudIdx, const float& intensityThresholdPercent) {
    return getCascadePointcloud(cascadeCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThresholdPercent);
}


octomap::OcTree coloradar::ColoradarPlusRun::buildLidarOctomap(
    const double& mapResolution,
    const float& lidarTotalHorizontalFov,
    const float& lidarTotalVerticalFov,
    const float& lidarMaxRange,
    Eigen::Affine3f lidarTransform
) {
    float maxRange = lidarMaxRange == 0 ? std::numeric_limits<float>::max() : lidarMaxRange;
    std::vector<octomath::Pose6D> gtPoses = getPoses<octomath::Pose6D>();
    std::vector<octomath::Pose6D> poses = interpolatePoses(gtPoses, poseTimestamps_, lidarTimestamps_);
    octomap::OcTree tree(mapResolution);

    for (size_t i = 0; i < lidarTimestamps_.size(); ++i) {
        OctoPointcloud cloud = getLidarPointCloud<coloradar::OctoPointcloud>(i);
        cloud.filterFov(lidarTotalHorizontalFov, lidarTotalVerticalFov, maxRange);
        cloud.transform(lidarTransform);
        cloud.transform(poses[i]);
        tree.insertPointCloud(cloud, poses[i].trans());
    }
    return tree;
}

void coloradar::ColoradarPlusRun::saveLidarOctomap(const octomap::OcTree& tree) {
    pcl::PointCloud<pcl::PointXYZI> treePcl;
    coloradar::octreeToPcl(tree, treePcl);
    coloradar::internal::createDirectoryIfNotExists(lidarMapsDirPath_);
    std::filesystem::path outputMapFile = lidarMapsDirPath_ / "map.pcd";
    pcl::io::savePCDFile(outputMapFile, treePcl);
}

pcl::PointCloud<pcl::PointXYZI> coloradar::ColoradarPlusRun::readLidarOctomap() {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::filesystem::path mapFilePath = lidarMapsDirPath_ / "map.pcd";
    coloradar::internal::checkPathExists(mapFilePath);
    pcl::io::loadPCDFile<pcl::PointXYZI>(mapFilePath.string(), cloud);
    return cloud;
}

void coloradar::ColoradarPlusRun::createLidarOctomap(
    const double& mapResolution,
    const float& lidarTotalHorizontalFov,
    const float& lidarTotalVerticalFov,
    const float& lidarMaxRange,
    Eigen::Affine3f lidarTransform
) {
    auto map = buildLidarOctomap(mapResolution, lidarTotalHorizontalFov, lidarTotalVerticalFov, lidarMaxRange, lidarTransform);
    saveLidarOctomap(map);
}

void coloradar::ColoradarPlusRun::sampleMapFrames(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const Eigen::Affine3f& mapPreTransform,
    std::vector<Eigen::Affine3f> poses
) {
    float maxRange = range == 0 ? std::numeric_limits<float>::max() : range;
    if (poses.empty())
        poses = getPoses<Eigen::Affine3f>();
    pcl::PointCloud<pcl::PointXYZI> origMapCloud = readLidarOctomap();
    pcl::PointCloud<pcl::PointXYZI> mapCloud;
    pcl::transformPointCloud(origMapCloud, mapCloud, mapPreTransform);

    for (size_t i = 0; i < poses.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI> centeredCloud;
        pcl::transformPointCloud(mapCloud, centeredCloud, poses[i]);
        filterFov(centeredCloud, horizontalFov, verticalFov, maxRange);
        std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(i) + ".pcd");
        pcl::io::savePCDFile(frameFilePath, centeredCloud);
    }
}

pcl::PointCloud<pcl::PointXYZI> coloradar::ColoradarPlusRun::readMapFrame(const int& frameIdx) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(frameIdx) + ".pcd");
    coloradar::internal::checkPathExists(frameFilePath);
    pcl::io::loadPCDFile<pcl::PointXYZI>(frameFilePath.string(), cloud);
    return cloud;
}

const std::string& coloradar::ColoradarPlusRun::exportToFile(
    const std::filesystem::path& filename,
    const bool& includeCascadeHeatmaps,
    const bool& includeCascadePointclouds,
    const int& cascadeAzimuthMaxBin,
    const int& cascadeElevationMaxBin,
    const int& cascadeRangeMaxBin,
    const bool& removeCascadeDopplerDim,
    const bool& collapseCascadeElevation,
    const int& collapseCascadeElevationMinBin,
    const int& collapseCascadeElevationMaxBin,
    const bool& includeLidarMap,
    const bool& includeMapFrames,
    const float& mapSampleTotalHorizontalFov,
    const float& mapSampleTotalVerticalFov,
    const float& mapSampleMaxRange,
    const Eigen::Affine3f& mapSamplingPreTransform,
    std::vector<Eigen::Affine3f> mapSamplingPoses,
    const bool& collapseLidarElevation,
    const float& collapseLidarElevationMin,
    const float& collapseLidarElevationMax,
    const bool& includeTruePoses,
    const bool& includeCascadePoses,
    const bool& includeLidarPoses,
    const bool& includeTrueTimestamps,
    const bool& includeCascadeTimestamps,
    const bool& includeLidarTimestamps,
    const bool& includeCascadeConfig
) const {
    return "";
}


coloradar::ColoradarRun::ColoradarRun(const std::filesystem::path& runPath, coloradar::RadarConfig* cascadeRadarConfig, coloradar::RadarConfig* singleChipRadarConfig) : coloradar::ColoradarPlusRun(runPath, cascadeRadarConfig), singleChipConfig_(singleChipRadarConfig) {
    singleChipScansDirPath_ = runDirPath_ / "cascade";
    coloradar::internal::checkPathExists(singleChipScansDirPath_);
    singleChipCubesDirPath_ = singleChipScansDirPath_ / "adc_samples";
    coloradar::internal::checkPathExists(singleChipCubesDirPath_);
    singleChipHeatmapsDirPath_ = singleChipScansDirPath_ / "heatmaps";
    coloradar::internal::checkPathExists(cascadeHeatmapsDirPath_);
    singleChipCloudsDirPath_ = singleChipScansDirPath_ / "pointclouds";

    singleChipCubeTimestamps_ = readTimestamps(singleChipCubesDirPath_ / "timestamps.txt");
    singleChipTimestamps_ = readTimestamps(singleChipHeatmapsDirPath_ / "timestamps.txt");
}

const std::vector<double>& coloradar::ColoradarRun::singleChipCubeTimestamps() const { return singleChipCubeTimestamps_; }
const std::vector<double>& coloradar::ColoradarRun::singleChipTimestamps() const { return singleChipTimestamps_; }

std::vector<int16_t> coloradar::ColoradarRun::getSingleChipDatacube(const std::filesystem::path& binFilePath) {
    return getDatacube(binFilePath, singleChipConfig_);
}
std::vector<int16_t> coloradar::ColoradarRun::getSingleChipDatacube(const int& cubeIdx) {
    return getSingleChipDatacube(singleChipCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}
std::vector<float> coloradar::ColoradarRun::getSingleChipHeatmap(const std::filesystem::path& binFilePath) {
    return getHeatmap(binFilePath, singleChipConfig_);
}
std::vector<float> coloradar::ColoradarRun::getSingleChipHeatmap(const int& hmIdx) {
    return getSingleChipHeatmap(singleChipHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarRun::getSingleChipPointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent) {
    return getRadarPointcloud(binFilePath, singleChipConfig_, intensityThresholdPercent);
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarRun::getSingleChipPointcloud(const int& cloudIdx, const float& intensityThresholdPercent) {
    return getSingleChipPointcloud(singleChipCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThresholdPercent);
}
//void coloradar::ColoradarRun::createSingleChipPointclouds(const float& intensityThresholdPercent) {
//    createRadarPointclouds(singleChipConfig_, singleChipHeatmapsDirPath_, singleChipCloudsDirPath_, intensityThresholdPercent);
//}
