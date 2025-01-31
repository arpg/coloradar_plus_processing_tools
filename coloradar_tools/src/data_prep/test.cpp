#include "coloradar_tools.h"


int main() {
    int start = 1012;
    coloradar::ColoradarDataset dataset("/home/arpg/coloradar");
    auto run = dataset.getRun("ec_hallways_run0");
    coloradar::OctoPointcloud cloud = run->getLidarPointCloud<coloradar::OctoPointcloud>(100);
    auto baseToLidarT = coloradar::internal::fromEigenPose<octomath::Pose6D>(dataset.lidarTransform());

    std::vector<octomath::Pose6D> gtPoses = run->getPoses<octomath::Pose6D>();
    std::vector<octomath::Pose6D> poses = run->interpolatePoses(gtPoses, run->poseTimestamps(), run->lidarTimestamps());

    for (size_t i = 0; i < std::min(gtPoses.size(), static_cast<size_t>(5)); ++i) {
        std::cout << "GT Pose " << i << ": "
                  << gtPoses[i].x() << " "
                  << gtPoses[i].y() << " "
                  << gtPoses[i].z() << "\n";
    }

    for (size_t i = 0; i < std::min(poses.size(), static_cast<size_t>(5)); ++i) {
        std::cout << "Interpolated Pose " << i << ": "
                  << poses[i].x() << " "
                  << poses[i].y() << " "
                  << poses[i].z() << "\n";
    }


//    auto mapToBaseT = poses[100];
//
//    std::cout << "Original Points in Sensor Frame:\n";
//    for (int i = 0; i < 3; ++i) {
//        auto point = cloud[start + i];
//        std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")\n";
//    }
//    std::cout << "bTs: (" << baseToLidarT.trans().x() << ", " << baseToLidarT.trans().y() << ", " << baseToLidarT.trans().z() << ")\n";
//    std::cout << "mTb: (" << mapToBaseT.trans().x() << ", " << mapToBaseT.trans().y() << ", " << mapToBaseT.trans().z() << ")\n";
//
//    auto mapToSensorT = mapToBaseT * baseToLidarT;
//    std::cout << "sensor origin in map: (" << mapToSensorT.trans().x() << ", " << mapToSensorT.trans().y() << ", " << mapToSensorT.trans().z() << ")\n";
//    cloud.transform(mapToSensorT);
//    std::cout << "Transformed Points in map frame:\n";
//    for (int i = 0; i < 3; ++i) {
//        auto point = cloud[start + i];
//        std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")\n";
//    }
//
//    Eigen::Affine3f baseToMapT = coloradar::internal::toEigenPose(mapToBaseT).inverse();
//    Eigen::Affine3f sensorToBaseT = dataset.lidarTransform().inverse();
//    auto sensorToMapT = sensorToBaseT * baseToMapT;
//    std::cout << "sTm: (" << sensorToMapT.translation().x() << ", " << sensorToMapT.translation().y() << ", " << sensorToMapT.translation().z() << ")\n";

    return 0;
}


//int main() {
//    coloradar::ColoradarDataset dataset("/home/arpg/coloradar");
//    auto runs = {dataset.getRun("ec_hallways_run0"), dataset.getRun("edgar_classroom_run0")};
//
//    bool includeCascadeHeatmaps = true;
//    bool includeCascadePointclouds = true;
//    int cascadeAzimuthMaxBin = 63;
//    int cascadeElevationMaxBin = 4;
//    int cascadeRangeMaxBin = 117;
//    bool collapseCascadeElevation = false;
//    int collapseCascadeElevationMinZ = -1;
//    int collapseCascadeElevationMaxZ = 1;
//    float cascadeCloudIntensityThresholdPercent = 0.0f;
//    bool removeCascadeDopplerDim = true;
//    bool includeLidarFrames = false;
//    float lidarFrameTotalHorizontalFov = 360.0f;
//    float lidarFrameTotalVerticalFov = 180.0f;
//    float lidarFrameMaxRange = 100.0f;
//    bool collapseLidarFrameElevation = false;
//    float collapseLidarFrameElevationMinZ = -1.0f;
//    float collapseLidarFrameElevationMaxZ = 1.0f;
//    bool includeLidarMap = true;
//    bool collapseMapElevation = false;
//    float collapseMapElevationMinZ = -1.0f;
//    float collapseMapElevationMaxZ = 1.0f;
//    Eigen::Affine3f lidarMapTransform = Eigen::Affine3f::Identity();
//    bool includeMapFrames = true;
//    float mapSampleTotalHorizontalFov = 360.0f;
//    float mapSampleTotalVerticalFov = 180.0f;
//    float mapSampleMaxRange = 100.0f;
//    Eigen::Affine3f mapSamplingPreTransform = Eigen::Affine3f::Identity();
//    std::vector<Eigen::Affine3f> mapSamplingPoses;
//    bool collapseMapSampleElevation = false;
//    float collapseMapSampleElevationMinZ = -1.0f;
//    float collapseMapSampleElevationMaxZ = 1.0f;
//    bool removeLidarIntensity = true;
//    bool includeTruePoses = false;
//    bool includeCascadePoses = true;
//    bool includeLidarPoses = false;
//    bool includeTrueTimestamps = false;
//    bool includeCascadeTimestamps = true;
//    bool includeLidarTimestamps = false;
//
//    try {
//        std::filesystem::path resultPath = dataset.exportToFile(
//            runs, "test_dataset.h5",
//            includeCascadeHeatmaps, includeCascadePointclouds, true,
//            cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin,
//            removeCascadeDopplerDim, collapseCascadeElevation, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, cascadeCloudIntensityThresholdPercent,
//            includeLidarFrames, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange, collapseLidarFrameElevation, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ,
//            includeLidarMap, collapseMapElevation, collapseMapElevationMinZ, collapseMapElevationMaxZ, lidarMapTransform,
//            includeMapFrames, mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingPreTransform, mapSamplingPoses, collapseMapSampleElevation, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ,
//            removeLidarIntensity,
//            includeTruePoses, includeCascadePoses, includeLidarPoses,
//            includeTrueTimestamps, includeCascadeTimestamps, includeLidarTimestamps
//        );
//
//        std::cout << "Export completed successfully. Dataset saved at: " << resultPath << std::endl;
//    } catch (const std::exception& e) {
//        std::cerr << "Error during export: " << e.what() << std::endl;
//    }
//
//    return 0;
//}
