#include "dataset.h"

#include <fstream>
#include <sstream>


coloradar::ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_radar.txt");
}

void coloradar::ColoradarPlusDataset::init(const std::filesystem::path& pathToDataset) {
    datasetDirPath_ = pathToDataset;
    coloradar::internal::checkPathExists(datasetDirPath_);
    calibDirPath_ = datasetDirPath_ / "calib";
    coloradar::internal::checkPathExists(calibDirPath_);
    transformsDirPath_ = calibDirPath_ / "transforms";
    coloradar::internal::checkPathExists(transformsDirPath_);
    runsDirPath_ = datasetDirPath_ / "kitti";
    coloradar::internal::checkPathExists(runsDirPath_);

    imuTransform_ = loadTransform(transformsDirPath_ / "base_to_imu.txt");
    lidarTransform_ = loadTransform(transformsDirPath_ / "base_to_lidar.txt");

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


void savePosesToHDF5(const std::string& name, H5::H5File& file, const std::vector<Eigen::Affine3f>& poses) {
    hsize_t dims[2] = { poses.size(), 7 };
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

std::vector<float> flattenHeatmap(const std::vector<float>& heatmap, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins, const int& numDims) {
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
    return reorganizedHeatmap;
}

std::vector<float> flattenRadarCloud(const pcl::PointCloud<coloradar::RadarPoint>& cloud, bool collapseElevation) {
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
    return data;
}

std::vector<float> flattenLidarCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud, bool collapseElevation, bool includeIntensity) {
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
    return data;
}

void saveVectorToHDF5(const std::string& name, H5::H5File& file, const std::vector<double>& vec) {
    hsize_t dims[1] = { vec.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
}

void saveHeatmapsToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatHeatmaps, const int& numFrames, const std::vector<hsize_t>& heatmapDims) {
    std::vector<hsize_t> dims;
    dims.push_back(static_cast<hsize_t>(numFrames));
    dims.insert(dims.end(), heatmapDims.begin(), heatmapDims.end());
    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    dataset.write(flatHeatmaps.data(), H5::PredType::NATIVE_FLOAT);
}


void saveCloudToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatCloud, const int& numDims) {
    int numPoints = flatCloud.size() / numDims;
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(flatCloud.data(), H5::PredType::NATIVE_FLOAT);
    }
}

void saveCloudsToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatClouds, const int& numFrames, const int& numDims) {
    int numPoints = flatClouds.size() / numFrames / numDims;
    hsize_t dims[3] = {numFrames, numPoints, numDims};
    H5::DataSpace dataspace(3, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(flatClouds.data(), H5::PredType::NATIVE_FLOAT);
    }
}


std::filesystem::path coloradar::ColoradarPlusDataset::exportToFile(
    std::vector<ColoradarPlusRun*> runs,
    std::filesystem::path destination,

    const bool& includeCascadeHeatmaps,
    const bool& includeCascadePointclouds,
    const bool& cascadePointcloudsInBaseFrame,
    const int& cascadeAzimuthMaxBin,
    const int& cascadeElevationMaxBin,
    const int& cascadeRangeMaxBin,
    const bool& removeCascadeDopplerDim,
    const bool& collapseCascadeElevation,
    const int& collapseCascadeElevationMinZ,
    const int& collapseCascadeElevationMaxZ,
    const float& cascadeCloudIntensityThresholdPercent,

    const bool& includeLidarFrames,
    const float& lidarFrameTotalHorizontalFov,
    const float& lidarFrameTotalVerticalFov,
    const float& lidarFrameMaxRange,
    const bool& collapseLidarFrameElevation,
    const float& collapseLidarFrameElevationMinZ,
    const float& collapseLidarFrameElevationMaxZ,

    const bool& includeLidarMap,
    const bool& collapseMapElevation,
    const float& collapseMapElevationMinZ,
    const float& collapseMapElevationMaxZ,
    const Eigen::Affine3f& lidarMapTransform,

    const bool& includeMapFrames,
    const float& mapSampleTotalHorizontalFov,
    const float& mapSampleTotalVerticalFov,
    const float& mapSampleMaxRange,
    const Eigen::Affine3f& mapSamplingSensorToBaseTransform,
    std::vector<Eigen::Affine3f> mapSamplingBasePoses,
    const bool& collapseMapSampleElevation,
    const float& collapseMapSampleElevationMinZ,
    const float& collapseMapSampleElevationMaxZ,

    const bool& removeLidarIntensity,

    const bool& includeTruePoses,
    const bool& includeCascadePoses,
    const bool& includeLidarPoses,
    const bool& includeTrueTimestamps,
    const bool& includeCascadeTimestamps,
    const bool& includeLidarTimestamps
)
{
    const std::string cascadeHeatmapContentName = "cascade_heatmaps", cascadeCloudContentName = "cascade_clouds",
                      lidarFrameContentName = "lidar_frames", lidarMapContentName = "lidar_map", mapFrameContentName = "lidar_map_frames",
                      truePosesContentName = "true_poses", cascadePosesContentName = "cascade_poses", lidarPosesContentName = "lidar_poses",
                      trueTimestampsContentName = "true_timestamps", cascadeTimestampsContentName = "cascade_timestamps", lidarTimestampsContentName = "lidar_timestamps";
    if (runs.empty()) {
        runs = getRuns();
    }
    if (destination.empty()) {
        destination = "dataset.h5";
    }
    std::filesystem::path destinationAbs = std::filesystem::absolute(destination);
    H5::H5File datasetFile(destinationAbs, H5F_ACC_TRUNC);
    Json::Value finalConfig;
    finalConfig["runs"] = Json::arrayValue;
    finalConfig["data_content"] = Json::arrayValue;
    finalConfig["radar_config"] = cascadeConfig_->toJson();
    if (includeCascadeHeatmaps) finalConfig["data_content"].append(cascadeHeatmapContentName);
    if (includeCascadePointclouds) finalConfig["data_content"].append(cascadeCloudContentName);
    if (includeLidarFrames) finalConfig["data_content"].append(lidarFrameContentName);
    if (includeLidarMap) finalConfig["data_content"].append(lidarMapContentName);
    if (includeMapFrames) finalConfig["data_content"].append(mapFrameContentName);
    if (includeTruePoses) finalConfig["data_content"].append(truePosesContentName);
    if (includeCascadePoses) finalConfig["data_content"].append(cascadePosesContentName);
    if (includeLidarPoses) finalConfig["data_content"].append(lidarPosesContentName);
    if (includeTrueTimestamps) finalConfig["data_content"].append(trueTimestampsContentName);
    if (includeCascadeTimestamps) finalConfig["data_content"].append(cascadeTimestampsContentName);
    if (includeLidarTimestamps) finalConfig["data_content"].append(lidarTimestampsContentName);

    int lidarFrameNumDims = (collapseLidarFrameElevation ? 3 : 4) - removeLidarIntensity,
        lidarMapNumDims = collapseMapElevation ? 3 : 4,
        mapFrameNumDims = collapseMapSampleElevation ? 3 : 4;
    int cascadeCloudNumDims = collapseCascadeElevation ? 3 : 4;
    int cascadeNumAzimuthBins = cascadeAzimuthMaxBin >= 0 && (cascadeAzimuthMaxBin + 1) * 2 < cascadeConfig_->numAzimuthBins ?
                                (cascadeAzimuthMaxBin + 1) * 2 :
                                cascadeConfig_->numAzimuthBins;
    int cascadeNumElevationBins = cascadeElevationMaxBin >= 0 && (cascadeElevationMaxBin + 1) * 2 < cascadeConfig_->numElevationBins ?
                                  (cascadeElevationMaxBin + 1) * 2 :
                                  cascadeConfig_->numElevationBins;
    int cascadeNumRangeBins = cascadeRangeMaxBin >= 0 && cascadeRangeMaxBin + 1 < cascadeConfig_->numPosRangeBins ?
                              cascadeRangeMaxBin + 1 :
                              cascadeConfig_->numPosRangeBins;
    int cascadeNumDims = removeCascadeDopplerDim ? 1 : 2;
    int elStartIdx = (cascadeConfig_->numElevationBins - cascadeNumElevationBins) / 2;
    int elEndIdx = elStartIdx + cascadeNumElevationBins;
    std::vector<float> elevationBins(cascadeConfig_->elevationBins.begin() + elStartIdx, cascadeConfig_->elevationBins.begin() + elEndIdx);
    float cascadeHorizontalFov, cascadeVerticalFov, cascadeRange;
    coloradar::convertRadarBinsToFov(cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_, cascadeHorizontalFov, cascadeVerticalFov, cascadeRange);
    std::vector<hsize_t> heatmapDims;
    if (cascadeNumAzimuthBins > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumAzimuthBins));
    if (cascadeNumRangeBins > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumRangeBins));
    if (cascadeNumElevationBins > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumElevationBins));
    if (cascadeNumDims > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumDims));
    if (heatmapDims.empty()) heatmapDims.push_back(1);

//    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_azimuth_bins"] = cascadeNumAzimuthBins;
//    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_elevation_bins"] = collapseCascadeElevation ? 1 : cascadeNumElevationBins;
//    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_range_bins"] = cascadeNumRangeBins;
//    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_dimensions"] = cascadeNumDim;

    for (auto* run : runs) {
        finalConfig["runs"].append(run->name);
        int numCascadeFrames = run->cascadeTimestamps().size();
        int numLidarFrames = run->lidarTimestamps().size();
        bool savedHeatmaps = false;
//        finalConfig["data_dimensions"]['cascade_num_frames'][run->name] = numCascadeFrames;
//        finalConfig["data_dimensions"]['lidar_num_frames'][run->name] = numLidarFrames;

        auto poses = run->getPoses<Eigen::Affine3f>();
        auto cascadePoses = run->interpolatePoses(poses, run->poseTimestamps(), run->cascadeTimestamps());
        auto lidarPoses = run->interpolatePoses(poses, run->poseTimestamps(), run->lidarTimestamps());

        if (includeCascadePointclouds) {
            std::vector<float> framesFlat;
            std::vector<float> heatmapsFlat;

            for (size_t i = 0; i < numCascadeFrames; ++i) {
                Eigen::Affine3f cascadeCloudTransform = cascadePointcloudsInBaseFrame ? cascadePoses[i] * cascadeTransform_ : Eigen::Affine3f::Identity();
                pcl::PointCloud<coloradar::RadarPoint> rawCascadeCloud;
                try {
                    rawCascadeCloud = run->getCascadePointcloud(i, cascadeCloudIntensityThresholdPercent);
                } catch (const std::filesystem::filesystem_error& e) {
                    auto heatmap = run->getCascadeHeatmap(i);
                    rawCascadeCloud = coloradar::heatmapToPointcloud(heatmap, cascadeConfig_, cascadeCloudIntensityThresholdPercent);
                    if (includeCascadeHeatmaps) {
                        // finalConfig["data_content"].append(cascadeHeatmapContentName);
                        heatmap = coloradar::clipHeatmapImage(heatmap, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_);
                        if (collapseCascadeElevation)
                            heatmap = coloradar::collapseHeatmapElevation(heatmap, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, elevationBins, cascadeNumAzimuthBins, cascadeNumRangeBins);
                        if (removeCascadeDopplerDim)
                            heatmap = coloradar::removeDoppler(heatmap);
                        auto heatmapFlat = flattenHeatmap(heatmap, cascadeNumAzimuthBins, collapseCascadeElevation? 1 : cascadeNumElevationBins, cascadeNumRangeBins, cascadeNumDims);
                        heatmapsFlat.insert(heatmapsFlat.end(), heatmapFlat.begin(), heatmapFlat.end());
                        savedHeatmaps = true;
                    }
                }
                pcl::PointCloud<coloradar::RadarPoint> cascadeCloud;
                pcl::transformPointCloud(rawCascadeCloud, cascadeCloud, cascadeCloudTransform);
                coloradar::filterFov(cascadeCloud, cascadeHorizontalFov, cascadeVerticalFov, cascadeRange);
                if (collapseCascadeElevation) {
                    coloradar::collapseElevation(cascadeCloud, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ);
                }
                auto cloudFlat = flattenRadarCloud(cascadeCloud, collapseCascadeElevation);
                framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(cascadeCloudContentName + "_" + run->name, datasetFile, framesFlat, numCascadeFrames, cascadeCloudNumDims);
            if (savedHeatmaps) saveHeatmapsToHDF5(cascadeHeatmapContentName + "_" + run->name, datasetFile, heatmapsFlat, numCascadeFrames, heatmapDims);
        }
        if (includeCascadeHeatmaps && !savedHeatmaps) {
            // finalConfig["data_content"].append(cascadeHeatmapContentName);
            std::vector<float> heatmapsFlat;
            for (size_t i = 0; i < numCascadeFrames; ++i) {
                auto heatmap = run->getCascadeHeatmap(i);
                heatmap = coloradar::clipHeatmapImage(heatmap, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_);
                if (collapseCascadeElevation)
                    heatmap = coloradar::collapseHeatmapElevation(heatmap, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, elevationBins, cascadeNumAzimuthBins, cascadeNumRangeBins);
                if (removeCascadeDopplerDim)
                    heatmap = coloradar::removeDoppler(heatmap);
                auto heatmapFlat = flattenHeatmap(heatmap, cascadeNumAzimuthBins, collapseCascadeElevation? 1 : cascadeNumElevationBins, cascadeNumRangeBins, cascadeNumDims);
                heatmapsFlat.insert(heatmapsFlat.end(), heatmapFlat.begin(), heatmapFlat.end());
            }
            saveHeatmapsToHDF5(cascadeHeatmapContentName + "_" + run->name, datasetFile, heatmapsFlat, numCascadeFrames, heatmapDims);
        }
        if (includeLidarFrames) {
            // finalConfig["data_content"].append(lidarFrameContentName);
            std::vector<float> framesFlat;
            for (size_t i = 0; i < numLidarFrames; ++i) {
                auto lidarFrame = run->getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(i);
                coloradar::filterFov(lidarFrame, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange);
                if (collapseLidarFrameElevation)
                    coloradar::collapseElevation(lidarFrame, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ);
                auto cloudFlat = flattenLidarCloud(lidarFrame, collapseLidarFrameElevation, !removeLidarIntensity);
                framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(lidarFrameContentName + "_" + run->name, datasetFile, framesFlat, numLidarFrames, lidarFrameNumDims);
        }
        if (includeLidarMap) {
            // std::cout << "lidarMapTransform: " << lidarMapTransform.translation().transpose() << ", Rotation (Quaternion): " << Eigen::Quaternionf(lidarMapTransform.rotation()).coeffs().transpose() << std::endl;
            // finalConfig["data_content"].append(lidarMapContentName);
            pcl::PointCloud<pcl::PointXYZI> mapRaw = run->readLidarOctomap();
            pcl::PointCloud<pcl::PointXYZI> map;
            pcl::transformPointCloud(mapRaw, map, lidarMapTransform);
//            for (size_t i = 0; i < 5; ++i)
//                std::cout << mapRaw.points[i].z << " ";
//            std::cout << std::endl;
//            for (size_t i = 0; i < 5; ++i)
//                std::cout << map.points[i].z << " ";
//            std::cout << std::endl;
            if (collapseMapElevation)
                coloradar::collapseElevation(map, collapseMapElevationMinZ, collapseMapElevationMaxZ);
            std::vector<float> mapFlat = flattenLidarCloud(map, collapseMapElevation, true);
            saveCloudToHDF5(lidarMapContentName + "_" + run->name, datasetFile, mapFlat, lidarMapNumDims);
        }
        if (includeMapFrames) {
            // finalConfig["data_content"].append(mapFrameContentName);
            std::vector<float> framesFlat;
            for (size_t i = 0; i < numCascadeFrames; ++i) {
                pcl::PointCloud<pcl::PointXYZI> lidarMapSample;
                try {
                    lidarMapSample = run->readMapFrame(i);
                } catch (const std::filesystem::filesystem_error& e) {
                    run->sampleMapFrames(mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingSensorToBaseTransform, mapSamplingBasePoses);
                    lidarMapSample = run->readMapFrame(i);
                }
                if (collapseMapSampleElevation)
                    coloradar::collapseElevation(lidarMapSample, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ);
                auto cloudFlat = flattenLidarCloud(lidarMapSample, collapseMapSampleElevation, true);
                framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(mapFrameContentName + "_" + run->name, datasetFile, framesFlat, numLidarFrames, lidarFrameNumDims);
        }
        if (includeTruePoses) {
            savePosesToHDF5(truePosesContentName + "_" + run->name, datasetFile, poses);
        }
        if (includeCascadePoses) {
            savePosesToHDF5(cascadePosesContentName + "_" + run->name, datasetFile, cascadePoses);
        }
        if (includeLidarPoses) {
            savePosesToHDF5(lidarPosesContentName + "_" + run->name, datasetFile, lidarPoses);
        }
        if (includeTrueTimestamps) {
            saveVectorToHDF5(trueTimestampsContentName + "_" + run->name, datasetFile, run->poseTimestamps());
        }
        if (includeCascadeTimestamps) {
            saveVectorToHDF5(cascadeTimestampsContentName + "_" + run->name, datasetFile, run->cascadeTimestamps());
        }
        if (includeLidarTimestamps) {
            saveVectorToHDF5(lidarTimestampsContentName + "_" + run->name, datasetFile, run->lidarTimestamps());
        }
    }

    std::string configString = Json::writeString(Json::StreamWriterBuilder(), finalConfig);
    H5::StrType strType(H5::PredType::C_S1, H5T_VARIABLE);
    H5::DataSpace dataspace(H5S_SCALAR);
    H5::DataSet configDataset = datasetFile.createDataSet("config", strType, dataspace);
    configDataset.write(configString, strType);

    return destinationAbs;
}
// {
//    if (runs.empty()) {
//        runs = getRuns();
//    }
//    if (destination.empty()) {
//        destination = "dataset_" + std::to_string(std::time(nullptr)) + ".h5";
//    }
//    std::filesystem::path destinationAbs = std::filesystem::absolute(destination);
//    H5::H5File datasetFile(destinationAbs, H5F_ACC_TRUNC);
//    Json::Value finalConfig;
//    finalConfig["runs"] = Json::arrayValue;
//    finalConfig["data_dimensions"] = Json::Value(Json::objectValue);
//
//    for (auto* run : runs) {
//        finalConfig["runs"].append(run->name);
//        std::cout << "Exporting run: " << run->name << std::endl;
//        std::filesystem::path runFile = run->exportToFile("", includeCascadeHeatmaps, includeCascadePointclouds,
//            cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, removeCascadeDopplerDim, collapseCascadeElevation, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, cascadeCloudIntensityThresholdPercent,
//            includeLidarFrames, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange, collapseLidarFrameElevation, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ,
//            includeLidarMap, collapseMapElevation, collapseMapElevationMinZ, collapseMapElevationMaxZ,
//            includeMapFrames, mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingPreTransform, mapSamplingPoses, collapseMapSampleElevation, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ,
//            removeLidarIntensity,
//            includeTruePoses, includeCascadePoses, includeLidarPoses,
//            includeTrueTimestamps, includeCascadeTimestamps, includeLidarTimestamps
//        );
//
//        H5::H5File runFileHandle(runFile, H5F_ACC_RDONLY);
//        Json::Value runConfig;
//        H5::DataSet configDataset = runFileHandle.openDataSet("config");
//        H5::DataSpace configSpace = configDataset.getSpace();
//        hsize_t configSize;
//        configSpace.getSimpleExtentDims(&configSize);
//        std::vector<char> configBuffer(configSize);
//        configDataset.read(configBuffer.data(), H5::PredType::C_S1);
//        std::istringstream configStream(std::string(configBuffer.begin(), configBuffer.end()));
//
//        configStream >> runConfig;
//        auto& dataContent = runConfig["data_content"];
//        auto& dataDimensions = runConfig["data_dimensions"];
//        finalConfig["data_content"] = runConfig["data_content"];
//        finalConfig["data_settings"] = runConfig["data_settings"];
//        finalConfig["data_dimensions"][run->name] = dataDimensions;
//        if (runConfig.isMember("radar_config")) {
//            finalConfig["radar_config"] = runConfig["radar_config"];
//        }
//
//        for (const auto& contentType : dataContent) {
//            std::string contentName = contentType.asString();
//            if (dataDimensions.isMember(contentName)) {
//                int numFrames = dataDimensions[contentName]["num_frames"].asInt();
//                std::vector<float> contentData;
//                for (int i = 0; i < numFrames; ++i) {
//                    std::string datasetName = contentName + "_" + std::to_string(i);
//                    H5::DataSet dataset = runFileHandle.openDataSet(datasetName);
//                    H5::DataSpace space = dataset.getSpace();
//                    std::vector<hsize_t> dims(space.getSimpleExtentNdims());
//                    space.getSimpleExtentDims(dims.data());
//                    size_t frameSize = std::accumulate(dims.begin(), dims.end(), 1, std::multiplies<hsize_t>());
//                    std::vector<float> frameData(frameSize);
//                    dataset.read(frameData.data(), H5::PredType::NATIVE_FLOAT);
//                    contentData.insert(contentData.end(), frameData.begin(), frameData.end());
//                }
//                std::vector<hsize_t> totalDims = {static_cast<hsize_t>(runs.size())};
//                for (auto dim : dataDimensions[contentName]["shape"]) {
//                    totalDims.push_back(dim.asUInt());
//                }
//                H5::DataSpace dataspace(totalDims.size(), totalDims.data());
//                H5::DataSet outputDataset = datasetFile.createDataSet(contentName + "_" + run->name, H5::PredType::NATIVE_FLOAT, dataspace);
//                outputDataset.write(contentData.data(), H5::PredType::NATIVE_FLOAT);
//            } else {
//                std::string datasetName = contentName + "_" + run->name;
//                H5::DataSet inputDataset = runFileHandle.openDataSet(contentName);
//                H5::DataSpace inputSpace = inputDataset.getSpace();
//                std::vector<hsize_t> dims(inputSpace.getSimpleExtentNdims());
//                inputSpace.getSimpleExtentDims(dims.data());
//                size_t dataSize = std::accumulate(dims.begin(), dims.end(), 1, std::multiplies<hsize_t>());
//                std::vector<float> buffer(dataSize);
//                inputDataset.read(buffer.data(), H5::PredType::NATIVE_FLOAT);
//                H5::DataSpace outputSpace(dims.size(), dims.data());
//                H5::DataSet outputDataset = datasetFile.createDataSet(datasetName, H5::PredType::NATIVE_FLOAT, outputSpace);
//                outputDataset.write(buffer.data(), H5::PredType::NATIVE_FLOAT);
//            }
//        }
//
//        std::filesystem::remove(runFile);
//    }
//    std::string configString = Json::writeString(Json::StreamWriterBuilder(), finalConfig);
//    H5::StrType strType(H5::PredType::C_S1, H5T_VARIABLE);
//    H5::DataSpace dataspace(H5S_SCALAR);
//    H5::DataSet configDataset = datasetFile.createDataSet("config", strType, dataspace);
//    configDataset.write(configString, strType);
//
//    return destinationAbs;
//}


coloradar::ColoradarDataset::ColoradarDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_cascade.txt");
    singleChipTransform_ = loadTransform(transformsDirPath_ / "base_to_single_chip.txt");
    singleChipConfig_ = new coloradar::SingleChipConfig(calibDirPath_);
}

coloradar::ColoradarPlusRun* coloradar::ColoradarDataset::getRun(const std::string& runName) {
    return new coloradar::ColoradarRun(runsDirPath_ / runName, cascadeConfig_, singleChipConfig_);
}

const Eigen::Affine3f& coloradar::ColoradarDataset::singleChipTransform() const { return singleChipTransform_; }
const coloradar::RadarConfig* coloradar::ColoradarDataset::singleChipConfig() const { return singleChipConfig_; }
