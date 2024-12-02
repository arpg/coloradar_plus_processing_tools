#include "coloradar_tools.h"


int main() {
    coloradar::ColoradarDataset dataset("/home/arpg/coloradar");
    auto runs = {dataset.getRun("ec_hallways_run0"), dataset.getRun("edgar_classroom_run0")};

    bool includeCascadeHeatmaps = true;
    bool includeCascadePointclouds = true;
    int cascadeAzimuthMaxBin = 63;
    int cascadeElevationMaxBin = 4;
    int cascadeRangeMaxBin = 117;
    bool collapseCascadeElevation = false;
    int collapseCascadeElevationMinZ = -1;
    int collapseCascadeElevationMaxZ = 1;
    float cascadeCloudIntensityThresholdPercent = 0.0f;
    bool removeCascadeDopplerDim = true;
    bool includeLidarFrames = false;
    float lidarFrameTotalHorizontalFov = 360.0f;
    float lidarFrameTotalVerticalFov = 180.0f;
    float lidarFrameMaxRange = 100.0f;
    bool collapseLidarFrameElevation = false;
    float collapseLidarFrameElevationMinZ = -1.0f;
    float collapseLidarFrameElevationMaxZ = 1.0f;
    bool includeLidarMap = true;
    bool collapseMapElevation = false;
    float collapseMapElevationMinZ = -1.0f;
    float collapseMapElevationMaxZ = 1.0f;
    Eigen::Affine3f lidarMapTransform = Eigen::Affine3f::Identity();
    bool includeMapFrames = true;
    float mapSampleTotalHorizontalFov = 360.0f;
    float mapSampleTotalVerticalFov = 180.0f;
    float mapSampleMaxRange = 100.0f;
    Eigen::Affine3f mapSamplingPreTransform = Eigen::Affine3f::Identity();
    std::vector<Eigen::Affine3f> mapSamplingPoses;
    bool collapseMapSampleElevation = false;
    float collapseMapSampleElevationMinZ = -1.0f;
    float collapseMapSampleElevationMaxZ = 1.0f;
    bool removeLidarIntensity = true;
    bool includeTruePoses = false;
    bool includeCascadePoses = true;
    bool includeLidarPoses = false;
    bool includeTrueTimestamps = false;
    bool includeCascadeTimestamps = true;
    bool includeLidarTimestamps = false;

    try {
        std::filesystem::path resultPath = dataset.exportToFile(
            runs, "test_dataset.h5",
            includeCascadeHeatmaps, includeCascadePointclouds, true,
            cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin,
            removeCascadeDopplerDim, collapseCascadeElevation, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, cascadeCloudIntensityThresholdPercent,
            includeLidarFrames, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange, collapseLidarFrameElevation, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ,
            includeLidarMap, collapseMapElevation, collapseMapElevationMinZ, collapseMapElevationMaxZ, lidarMapTransform,
            includeMapFrames, mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingPreTransform, mapSamplingPoses, collapseMapSampleElevation, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ,
            removeLidarIntensity,
            includeTruePoses, includeCascadePoses, includeLidarPoses,
            includeTrueTimestamps, includeCascadeTimestamps, includeLidarTimestamps
        );

        std::cout << "Export completed successfully. Dataset saved at: " << resultPath << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error during export: " << e.what() << std::endl;
    }

    return 0;
}
