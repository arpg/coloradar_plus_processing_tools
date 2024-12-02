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
        throw std::filesystem::filesystem_error("Failed to open file", binFilePath, std::make_error_code(std::errc::no_such_file_or_directory));
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
    Eigen::Affine3f lidarToBaseTransform
) {
    float maxRange = lidarMaxRange == 0 ? std::numeric_limits<float>::max() : lidarMaxRange;
    std::vector<octomath::Pose6D> gtPoses = getPoses<octomath::Pose6D>();
    std::vector<octomath::Pose6D> poses = interpolatePoses(gtPoses, poseTimestamps_, lidarTimestamps_);
    auto lidarToBase = coloradar::internal::fromEigenPose<octomath::Pose6D>(lidarToBaseTransform);
    octomap::OcTree tree(mapResolution);

    for (size_t i = 0; i < lidarTimestamps_.size(); ++i) {
        OctoPointcloud cloud = getLidarPointCloud<coloradar::OctoPointcloud>(i);
        cloud.filterFov(lidarTotalHorizontalFov, lidarTotalVerticalFov, maxRange);
        auto frameTransform = poses[i] * lidarToBase;
        cloud.transform(frameTransform);
        tree.insertPointCloud(cloud, frameTransform.trans());
    }
    return tree;
}

void coloradar::ColoradarPlusRun::saveLidarOctomap(const octomap::OcTree& tree) {
    pcl::PointCloud<pcl::PointXYZI> treePcl;
    coloradar::octreeToPcl(tree, treePcl);
    coloradar::internal::createDirectoryIfNotExists(lidarMapsDirPath_);
    std::filesystem::path outputMapFile = lidarMapsDirPath_ / "map.pcd";
    if (treePcl.size() == 0) {
        treePcl.height = 1;
        treePcl.is_dense = true;
    }
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
    Eigen::Affine3f lidarToBaseTransform
) {
    auto map = buildLidarOctomap(mapResolution, lidarTotalHorizontalFov, lidarTotalVerticalFov, lidarMaxRange, lidarToBaseTransform);
    saveLidarOctomap(map);
}

void coloradar::ColoradarPlusRun::sampleMapFrames(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const Eigen::Affine3f& sensorToBaseTransform,
    std::vector<Eigen::Affine3f> basePoses
) {
    for (const auto& entry : std::filesystem::directory_iterator(lidarMapsDirPath_)) {
        if (entry.is_regular_file() && entry.path().filename() != "map.pcd") {
            std::filesystem::remove(entry.path());
        }
    }
    float maxRange = range == 0 ? std::numeric_limits<float>::max() : range;
    if (basePoses.empty())
        basePoses = getPoses<Eigen::Affine3f>();
    pcl::PointCloud<pcl::PointXYZI> mapCloud = readLidarOctomap();
    for (size_t i = 0; i < basePoses.size(); ++i) {
        Eigen::Affine3f radarPose = basePoses[i] * sensorToBaseTransform.inverse();
        pcl::PointCloud<pcl::PointXYZI> centeredCloud;
        pcl::transformPointCloud(mapCloud, centeredCloud, radarPose);
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



void coloradar::ColoradarPlusRun::saveVectorToHDF5(const std::string& name, H5::H5File& file, const std::vector<double>& vec) {
    hsize_t dims[1] = { vec.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
}

void coloradar::ColoradarPlusRun::savePosesToHDF5(const std::string& name, H5::H5File& file, const std::vector<Eigen::Affine3f>& poses) {
    hsize_t dims[2] = { poses.size(), 7 };  // N * (x, y, z, qx, qy, qz, qw)
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
    std::vector<float> poseData;
    poseData.reserve(poses.size() * 7);
    for (const auto& pose : poses) {
        poseData.push_back(pose.translation().x());
        poseData.push_back(pose.translation().y());
        poseData.push_back(pose.translation().z());
        Eigen::Quaternionf rot(pose.rotation());
        poseData.push_back(rot.x());
        poseData.push_back(rot.y());
        poseData.push_back(rot.z());
        poseData.push_back(rot.w());
    }
    dataset.write(poseData.data(), H5::PredType::NATIVE_FLOAT);
}

void coloradar::ColoradarPlusRun::saveHeatmapToHDF5(const int& idx, H5::H5File& file, const std::vector<float>& heatmap, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins, const int& numDims) {
    const size_t expectedSize = numElevationBins * numAzimuthBins * numRangeBins * numDims;
    if (heatmap.size() != expectedSize) {
        throw std::runtime_error("Heatmap size does not match the expected dimensions. Expected size: " + std::to_string(expectedSize) + ", Actual size: " + std::to_string(heatmap.size()));
    }
    std::vector<float> reorganizedHeatmap(expectedSize);
    for (int el = 0; el < numElevationBins; ++el) {
        for (int az = 0; az < numAzimuthBins; ++az) {
            for (int r = 0; r < numRangeBins; ++r) {
                for (int d = 0; d < numDims; ++d) {
                    size_t originalIndex = (((el * numAzimuthBins + az) * numRangeBins) + r) * numDims + d;
                    size_t reorganizedIndex = (((az * numRangeBins + r) * numElevationBins) + el) * numDims + d;
                    reorganizedHeatmap[reorganizedIndex] = heatmap[originalIndex];
                }
            }
        }
    }
    std::vector<hsize_t> dims;
    if (numAzimuthBins > 1) dims.push_back(static_cast<hsize_t>(numAzimuthBins));
    if (numRangeBins > 1) dims.push_back(static_cast<hsize_t>(numRangeBins));
    if (numElevationBins > 1) dims.push_back(static_cast<hsize_t>(numElevationBins));
    if (numDims > 1) dims.push_back(static_cast<hsize_t>(numDims));
    if (dims.empty()) dims.push_back(1);

    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet("heatmap_" + std::to_string(idx), datatype, dataspace);
    dataset.write(reorganizedHeatmap.data(), H5::PredType::NATIVE_FLOAT);
}

void coloradar::ColoradarPlusRun::saveRadarCloudToHDF5(const int& idx, H5::H5File& file, const pcl::PointCloud<coloradar::RadarPoint>& cloud, bool collapseElevation) {
    size_t numPoints = cloud.size();
    size_t numDims = collapseElevation ? 3 : 4;
    std::vector<float> data;
    if (numPoints > 0) {
        data.resize(numPoints * numDims);
        if (collapseElevation) {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].intensity;
            }
        } else {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].z;
                data[i * numDims + 3] = cloud[i].intensity;
            }
        }
    }
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    std::string datasetName = "radar_cloud_" + std::to_string(idx);
    H5::DataSet dataset = file.createDataSet(datasetName, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(data.data(), H5::PredType::NATIVE_FLOAT);
    }
}

void coloradar::ColoradarPlusRun::saveLidarCloudToHDF5(const std::string& name, H5::H5File& file, const pcl::PointCloud<pcl::PointXYZI>& cloud, bool includeIntensity, bool collapseElevation) {
    size_t numPoints = cloud.size();
    size_t numDims = collapseElevation ? 3 : 4;
    if (!includeIntensity) numDims--;
    std::vector<float> data;
    if (numPoints > 0) {
        data.resize(numPoints * numDims);
        if (collapseElevation) {
            if (includeIntensity) {
                for (size_t i = 0; i < numPoints; ++i) {
                    data[i * numDims + 0] = cloud[i].x;
                    data[i * numDims + 1] = cloud[i].y;
                    data[i * numDims + 2] = cloud[i].intensity;
                }
            } else {
                for (size_t i = 0; i < numPoints; ++i) {
                    data[i * numDims + 0] = cloud[i].x;
                    data[i * numDims + 1] = cloud[i].y;
                }
            }
        } else if (includeIntensity) {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].z;
                data[i * numDims + 3] = cloud[i].intensity;
            }
        } else {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].z;
            }
        }
    }
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(data.data(), H5::PredType::NATIVE_FLOAT);
    }
}

//std::filesystem::path coloradar::ColoradarPlusRun::exportToFile(
//    std::filesystem::path destination,
//    const bool& includeCascadeHeatmaps,
//    const bool& includeCascadePointclouds,
//    const int& cascadeAzimuthMaxBin,
//    const int& cascadeElevationMaxBin,
//    const int& cascadeRangeMaxBin,
//    const bool& removeCascadeDopplerDim,
//    const bool& collapseCascadeElevation,
//    const int& collapseCascadeElevationMinZ,
//    const int& collapseCascadeElevationMaxZ,
//    const float& cascadeCloudIntensityThresholdPercent,
//
//    const bool& includeLidarFrames,
//    const float& lidarFrameTotalHorizontalFov,
//    const float& lidarFrameTotalVerticalFov,
//    const float& lidarFrameMaxRange,
//    const bool& collapseLidarFrameElevation,
//    const float& collapseLidarFrameElevationMinZ,
//    const float& collapseLidarFrameElevationMaxZ,
//
//    const bool& includeLidarMap,
//    const bool& collapseMapElevation,
//    const float& collapseMapElevationMinZ,
//    const float& collapseMapElevationMaxZ,
//
//    const bool& includeMapFrames,
//    const float& mapSampleTotalHorizontalFov,
//    const float& mapSampleTotalVerticalFov,
//    const float& mapSampleMaxRange,
//    const Eigen::Affine3f& mapSamplingPreTransform,
//    std::vector<Eigen::Affine3f> mapSamplingPoses,
//    const bool& collapseMapSampleElevation,
//    const float& collapseMapSampleElevationMinZ,
//    const float& collapseMapSampleElevationMaxZ,
//
//    const bool& removeLidarIntensity,
//
//    const bool& includeTruePoses,
//    const bool& includeCascadePoses,
//    const bool& includeLidarPoses,
//    const bool& includeTrueTimestamps,
//    const bool& includeCascadeTimestamps,
//    const bool& includeLidarTimestamps
//) {
//    if (destination.empty()) {
//        destination = name + ".h5";
//    }
//    std::filesystem::path destinationAbs = std::filesystem::absolute(destination);
//    H5::H5File file(destinationAbs, H5F_ACC_TRUNC);
//
//    int cascadeNumAzimuthBins = cascadeAzimuthMaxBin >= 0 && (cascadeAzimuthMaxBin + 1) * 2 < cascadeConfig_->numAzimuthBins ? (cascadeAzimuthMaxBin + 1) * 2 : cascadeConfig_->numAzimuthBins;
//    int cascadeNumElevationBins = cascadeElevationMaxBin >= 0 && (cascadeElevationMaxBin + 1) * 2 < cascadeConfig_->numElevationBins ? (cascadeElevationMaxBin + 1) * 2 : cascadeConfig_->numElevationBins;
//    int cascadeNumRangeBins = cascadeRangeMaxBin >= 0 && cascadeRangeMaxBin + 1 < cascadeConfig_->numPosRangeBins ? cascadeRangeMaxBin + 1 : cascadeConfig_->numPosRangeBins;
//    int cascadeNumDim = removeCascadeDopplerDim ? 1 : 2;
//
//    int elStartIdx = (cascadeConfig_->numElevationBins - cascadeNumElevationBins) / 2;
//    int elEndIdx = elStartIdx + cascadeNumElevationBins;
//    std::vector<float> elevationBins(cascadeConfig_->elevationBins.begin() + elStartIdx, cascadeConfig_->elevationBins.begin() + elEndIdx);
//
//    float cascadeHorizontalFov, cascadeVerticalFov, cascadeRange;
//    coloradar::convertRadarBinsToFov(cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_, cascadeHorizontalFov, cascadeVerticalFov, cascadeRange);
//    std::cout << "Using cascade horizontal FOV " << cascadeHorizontalFov << ", vertical FOV " << cascadeVerticalFov << ", range " << cascadeRange << std::endl;
//
//    int numCascadeFrames = cascadeTimestamps_.size();
//    int numLidarFrames = lidarTimestamps_.size();
//    bool savedHeatmaps = false;
//
//    if (includeCascadePointclouds) {
//        for (size_t i = 0; i < numCascadeFrames; ++i) {
//            pcl::PointCloud<coloradar::RadarPoint> cascadeCloud;
//            try {
//                cascadeCloud = getCascadePointcloud(i, cascadeCloudIntensityThresholdPercent);
//            } catch (const std::filesystem::filesystem_error& e) {
//                auto heatmap = getCascadeHeatmap(i);
//                cascadeCloud = coloradar::heatmapToPointcloud(heatmap, cascadeConfig_, cascadeCloudIntensityThresholdPercent);
//
//                if (includeCascadeHeatmaps) {
//                    heatmap = coloradar::clipHeatmapImage(heatmap, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_);
//                    if (collapseCascadeElevation)
//                        heatmap = coloradar::collapseHeatmapElevation(heatmap, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, elevationBins, cascadeNumAzimuthBins, cascadeNumRangeBins);
//                    if (removeCascadeDopplerDim)
//                        heatmap = coloradar::removeDoppler(heatmap);
//                    saveHeatmapToHDF5(i, file, heatmap, cascadeNumAzimuthBins, collapseCascadeElevation? 1 : cascadeNumElevationBins, cascadeNumRangeBins, cascadeNumDim);
//                    savedHeatmaps = true;
//                }
//            }
//            coloradar::filterFov(cascadeCloud, cascadeHorizontalFov, cascadeVerticalFov, cascadeRange);
//            if (collapseCascadeElevation) {
//                coloradar::collapseElevation(cascadeCloud, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ);
//            }
//            saveRadarCloudToHDF5(i, file, cascadeCloud, collapseCascadeElevation);
//        }
//    }
//    if (includeCascadeHeatmaps && !savedHeatmaps) {
//        for (size_t i = 0; i < numCascadeFrames; ++i) {
//            auto heatmap = getCascadeHeatmap(i);
//            heatmap = coloradar::clipHeatmapImage(heatmap, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_);
//            if (collapseCascadeElevation)
//                heatmap = coloradar::collapseHeatmapElevation(heatmap, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, elevationBins, cascadeNumAzimuthBins, cascadeNumRangeBins);
//            if (removeCascadeDopplerDim)
//                heatmap = coloradar::removeDoppler(heatmap);
//            saveHeatmapToHDF5(i, file, heatmap, cascadeNumAzimuthBins, collapseCascadeElevation? 1 : cascadeNumElevationBins, cascadeNumRangeBins, cascadeNumDim);
//        }
//    }
//    if (includeLidarFrames) {
//        for (size_t i = 0; i < numLidarFrames; ++i) {
//            auto lidarFrame = getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(i);
//            coloradar::filterFov(lidarFrame, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange);
//            if (collapseLidarFrameElevation)
//                coloradar::collapseElevation(lidarFrame, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ);
//            saveLidarCloudToHDF5("lidar_frame_" + std::to_string(i), file, lidarFrame, collapseLidarFrameElevation, !removeLidarIntensity);
//        }
//    }
//    if (includeLidarMap) {
//        auto map = readLidarOctomap();
//        if (collapseMapElevation)
//            coloradar::collapseElevation(map, collapseMapElevationMinZ, collapseMapElevationMaxZ);
//        saveLidarCloudToHDF5("lidar_map", file, map, collapseMapElevation, true);
//    }
//    if (includeMapFrames) {
//        for (size_t i = 0; i < numCascadeFrames; ++i) {
//            pcl::PointCloud<pcl::PointXYZI> lidarMapSample;
//            try {
//                lidarMapSample = readMapFrame(i);
//            } catch (const std::filesystem::filesystem_error& e) {
//                std::cout << "Sampling map frames..." << std::endl;
//                sampleMapFrames(mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingPreTransform, mapSamplingPoses);
//                lidarMapSample = readMapFrame(i);
//            }
//            if (collapseMapSampleElevation)
//                coloradar::collapseElevation(lidarMapSample, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ);
//            saveLidarCloudToHDF5("map_frame_" + std::to_string(i), file, lidarMapSample, collapseMapSampleElevation, true);
//        }
//    }
//    auto poses = getPoses<Eigen::Affine3f>();
//    if (includeTruePoses) {
//        savePosesToHDF5("true_poses", file, poses);
//    }
//    if (includeCascadePoses) {
//        auto cascadePoses = interpolatePoses(poses, poseTimestamps_, cascadeTimestamps_);
//        savePosesToHDF5("cascade_poses", file, cascadePoses);
//    }
//    if (includeLidarPoses) {
//        auto lidarPoses = interpolatePoses(poses, poseTimestamps_, lidarTimestamps_);
//        savePosesToHDF5("lidar_poses", file, lidarPoses);
//    }
//    if (includeTrueTimestamps) {
//        saveVectorToHDF5("true_timestamps", file, poseTimestamps_);
//    }
//    if (includeCascadeTimestamps) {
//        saveVectorToHDF5("cascade_timestamps", file, cascadeTimestamps_);
//    }
//    if (includeLidarTimestamps) {
//        saveVectorToHDF5("lidar_timestamps", file, lidarTimestamps_);
//    }
//
//    Json::Value config;
//    config["data_content"] = Json::arrayValue;
//    if (includeCascadeHeatmaps) config["data_content"].append("heatmap");
//    if (includeCascadePointclouds) config["data_content"].append("radar_cloud");
//    if (includeLidarFrames) config["data_content"].append("lidar_frame");
//    if (includeLidarMap) config["data_content"].append("lidar_map");
//    if (includeMapFrames) config["data_content"].append("map_frame");
//    if (includeTruePoses) config["data_content"].append("true_poses");
//    if (includeCascadePoses) config["data_content"].append("cascade_poses");
//    if (includeLidarPoses) config["data_content"].append("lidar_poses");
//    if (includeTrueTimestamps) config["data_content"].append("true_timestamps");
//    if (includeCascadeTimestamps) config["data_content"].append("cascade_timestamps");
//    if (includeLidarTimestamps) config["data_content"].append("lidar_timestamps");
//    config["data_settings"]["cascade_elevation_limits"] = Json::arrayValue;
//    config["data_settings"]["cascade_elevation_limits"].append(collapseCascadeElevationMinZ);
//    config["data_settings"]["cascade_elevation_limits"].append(collapseCascadeElevationMaxZ);
//    config["data_settings"]["cascade_intensity_threshold"] = cascadeCloudIntensityThresholdPercent;
//    config["data_settings"]["lidar_frame_fov"] = Json::arrayValue;
//    config["data_settings"]["lidar_frame_fov"].append(lidarFrameTotalHorizontalFov);
//    config["data_settings"]["lidar_frame_fov"].append(lidarFrameTotalVerticalFov);
//    config["data_settings"]["lidar_frame_elevation_limits"] = Json::arrayValue;
//    config["data_settings"]["lidar_frame_elevation_limits"].append(collapseLidarFrameElevationMinZ);
//    config["data_settings"]["lidar_frame_elevation_limits"].append(collapseLidarFrameElevationMaxZ);
//    config["data_settings"]["lidar_map_elevation_limits"] = Json::arrayValue;
//    config["data_settings"]["lidar_map_elevation_limits"].append(collapseMapElevationMinZ);
//    config["data_settings"]["lidar_map_elevation_limits"].append(collapseMapElevationMaxZ);
//    config["data_settings"]["map_sample_fov"] = Json::arrayValue;
//    config["data_settings"]["map_sample_fov"].append(mapSampleTotalHorizontalFov);
//    config["data_settings"]["map_sample_fov"].append(mapSampleTotalVerticalFov);
//    config["data_settings"]["map_sample_elevation_limits"] = Json::arrayValue;
//    config["data_settings"]["map_sample_elevation_limits"].append(collapseMapSampleElevationMinZ);
//    config["data_settings"]["map_sample_elevation_limits"].append(collapseMapSampleElevationMaxZ);
//    if (includeCascadeHeatmaps) {
//        config["data_dimensions"]["heatmap"]["num_frames"] = cascadeTimestamps_.size();
//        config["data_dimensions"]["heatmap"]["num_azimuth_bins"] = cascadeNumAzimuthBins;
//        config["data_dimensions"]["heatmap"]["num_elevation_bins"] = collapseCascadeElevation ? 1 : cascadeNumElevationBins;
//        config["data_dimensions"]["heatmap"]["num_range_bins"] = cascadeNumRangeBins;
//        config["data_dimensions"]["heatmap"]["num_dimensions"] = cascadeNumDim;
//    }
//    if (includeCascadePointclouds) {
//        config["data_dimensions"]["radar_cloud"]["num_frames"] = cascadeTimestamps_.size();
//    }
//    if (includeLidarFrames) {
//        config["data_dimensions"]["lidar_frame"]["num_frames"] = lidarTimestamps_.size();
//    }
//    if (includeMapFrames) {
//        config["data_dimensions"]["map_frame"]["num_frames"] = cascadeTimestamps_.size();
//    }
////    if (includeCascadeHeatmaps || includeCascadePointclouds) {
////        config["radar_config"] = Json::Value(cascadeConfig_->toJson());
////    }
//    Json::StreamWriterBuilder writer;
//    std::string configString = Json::writeString(writer, config);
//    hsize_t configDims[1] = {configString.size()};
//    H5::DataSpace configSpace(1, configDims);
//    H5::DataSet configDataset = file.createDataSet("config", H5::PredType::C_S1, configSpace);
//    configDataset.write(configString.c_str(), H5::PredType::C_S1);
//
//    return destinationAbs;
//}


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
