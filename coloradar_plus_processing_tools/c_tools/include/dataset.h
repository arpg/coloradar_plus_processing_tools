#ifndef DATASET_H
#define DATASET_H

#include "coloradar_run.h"


namespace coloradar {

class ColoradarPlusDataset {
protected:
    std::filesystem::path datasetDirPath_;
    std::filesystem::path calibDirPath_;
    std::filesystem::path transformsDirPath_;
    std::filesystem::path runsDirPath_;

    Eigen::Affine3f imuTransform_;
    Eigen::Affine3f lidarTransform_;
    Eigen::Affine3f cascadeTransform_;

    RadarConfig* cascadeConfig_;

    ColoradarPlusDataset() = default;
    void init(const std::filesystem::path& pathToDataset);
    Eigen::Affine3f loadTransform(const std::filesystem::path& filePath);

public:
    ColoradarPlusDataset(const std::filesystem::path& pathToDataset);

    std::vector<std::string> listRuns();
    std::vector<ColoradarPlusRun*> getRuns();

    virtual ColoradarPlusRun* getRun(const std::string& runName);

    const Eigen::Affine3f& imuTransform() const;
    const Eigen::Affine3f& lidarTransform() const;
    const Eigen::Affine3f& cascadeTransform() const;
    const RadarConfig* cascadeConfig() const;

//    const std::string export(
//        const std::vector<ColoradarPlusRun*>& runs,
//        const std::filesystem::path& destinationDir = "",
//
//        const bool& includeCascadeHeatmaps = false,
//        const bool& includeCascadePointclouds = false,
//        const int& cascadeAzimuthMaxBin = -1,
//        const int& cascadeElevationMaxBin = -1,
//        const int& cascadeRangeMaxBin = -1,
//        const bool& removeCascadeDopplerDim = false,
//        const bool& collapseCascadeElevation = false,
//        const int& collapseCascadeElevationMinBin = -1,
//        const int& collapseCascadeElevationMaxBin = -1,
//
//        const bool& includeLidarMap = false,
//        const bool& includeMapFrames = false,
//        const float& mapSampleTotalHorizontalFov = 360,
//        const float& mapSampleTotalVerticalFov = 180,
//        const float& mapSampleMaxRange = 100,
//        const Eigen::Affine3f& mapSamplingPreTransform = Eigen::Affine3f::Identity(),
//        std::vector<Eigen::Affine3f> mapSamplingPoses = {},
//        const bool& collapseLidarElevation = false,
//        const float& collapseLidarElevationMin = -1,
//        const float& collapseLidarElevationMax = -1,
//
//        const bool& includeTruePoses = false,
//        const bool& includeCascadePoses = false,
//        const bool& includeLidarPoses = false,
//        const bool& includeTrueTimestamps = false,
//        const bool& includeCascadeTimestamps = false,
//        const bool& includeLidarTimestamps = false,
//
//        const bool& includeCascadeConfig = false
//    ) const;
};

class ColoradarDataset : public ColoradarPlusDataset {
protected:
    Eigen::Affine3f singleChipTransform_;

    RadarConfig* singleChipConfig_;

public:
    ColoradarDataset(const std::filesystem::path& pathToDataset);

    virtual ColoradarPlusRun* getRun(const std::string& runName) override;

    const Eigen::Affine3f& singleChipTransform() const;
    const RadarConfig* singleChipConfig() const;
};

}

#endif
