#ifndef COLORADAR_RUN_H
#define COLORADAR_RUN_H

#include "utils.h"


namespace coloradar {

class ColoradarPlusRun {
protected:
    std::filesystem::path runDirPath_;
    std::filesystem::path posesDirPath_;
    std::filesystem::path imuDirPath_;
    std::filesystem::path lidarScansDirPath_;
    std::filesystem::path lidarCloudsDirPath_;
    std::filesystem::path lidarMapsDirPath_;
    std::filesystem::path cascadeScansDirPath_;
    std::filesystem::path cascadeCubesDirPath_;
    std::filesystem::path cascadeHeatmapsDirPath_;
    std::filesystem::path cascadeCloudsDirPath_;

    std::vector<double> poseTimestamps_;
    std::vector<double> imuTimestamps_;
    std::vector<double> lidarTimestamps_;
    std::vector<double> cascadeCubeTimestamps_;
    std::vector<double> cascadeTimestamps_;
    std::vector<double> readTimestamps(const std::filesystem::path& path);

    RadarConfig* cascadeConfig_;
    std::vector<int16_t> getDatacube(const std::filesystem::path& binFilePath, RadarConfig* config);
    std::vector<float> getHeatmap(const std::filesystem::path& binFilePath, RadarConfig* config);
    void createRadarPointclouds(RadarConfig* config, const std::filesystem::path& heatmapDirPath, const std::filesystem::path& pointcloudDirPath, const float& intensityThresholdPercent);
    pcl::PointCloud<RadarPoint> getRadarPointcloud(const std::filesystem::path& binFilePath, RadarConfig* config, const float& intensityThresholdPercent);

public:
    const std::string name;

    ColoradarPlusRun(const std::filesystem::path& runPath, RadarConfig* cascadeRadarConfig);

    const std::vector<double>& poseTimestamps() const;
    const std::vector<double>& imuTimestamps() const;
    const std::vector<double>& lidarTimestamps() const;
    const std::vector<double>& cascadeCubeTimestamps() const;
    const std::vector<double>& cascadeTimestamps() const;

    std::vector<int16_t> getCascadeDatacube(const std::filesystem::path& binFilePath);
    std::vector<int16_t> getCascadeDatacube(const int& cubeIdx);
    std::vector<float> getCascadeHeatmap(const std::filesystem::path& binFilePath);
    std::vector<float> getCascadeHeatmap(const int& hmIdx);

    void createCascadePointclouds(const float& intensityThresholdPercent);
    pcl::PointCloud<RadarPoint> getCascadePointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent);
    pcl::PointCloud<RadarPoint> getCascadePointcloud(const int& cloudIdx, const float& intensityThresholdPercent);

    template<PoseType PoseT> std::vector<PoseT> getPoses();
    template<coloradar::PoseType PoseT> std::vector<PoseT> interpolatePoses(const std::vector<PoseT>& poses, const std::vector<double>& poseTimestamps, const std::vector<double>& targetTimestamps);

    template<PclCloudType CloudT> CloudT getLidarPointCloud(const std::filesystem::path& binPath);
    template<OctomapCloudType CloudT> CloudT getLidarPointCloud(const std::filesystem::path& binPath);
    template<CloudType CloudT> CloudT getLidarPointCloud(const int& cloudIdx);

    octomap::OcTree buildLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f lidarTransform = Eigen::Affine3f::Identity()
    );
    void saveLidarOctomap(const octomap::OcTree& tree);
    pcl::PointCloud<pcl::PointXYZI> readLidarOctomap();

    void sampleMapFrames(
        const float& horizontalFov,
        const float& verticalFov,
        const float& range,
        const Eigen::Affine3f& mapPreTransform = Eigen::Affine3f::Identity(),
        std::vector<octomath::Pose6D> poses = {}
    );
    pcl::PointCloud<pcl::PointXYZI> readMapFrame(const int& frameIdx);

    virtual ~ColoradarPlusRun() = default;
};


class ColoradarRun : public ColoradarPlusRun {
protected:
    std::filesystem::path singleChipScansDirPath_;
    std::filesystem::path singleChipCubesDirPath_;
    std::filesystem::path singleChipHeatmapsDirPath_;
    std::filesystem::path singleChipCloudsDirPath_;

    std::vector<double> singleChipCubeTimestamps_;
    std::vector<double> singleChipTimestamps_;

    RadarConfig* singleChipConfig_;

public:
    ColoradarRun(const std::filesystem::path& runPath, RadarConfig* cascadeRadarConfig, RadarConfig* singleChipRadarConfig);

    const std::vector<double>& singleChipCubeTimestamps() const;
    const std::vector<double>& singleChipTimestamps() const;

    std::vector<int16_t> getSingleChipDatacube(const std::filesystem::path& binFilePath);
    std::vector<int16_t> getSingleChipDatacube(const int& cubeIdx);
    std::vector<float> getSingleChipHeatmap(const std::filesystem::path& binFilePath);
    std::vector<float> getSingleChipHeatmap(const int& hmIdx);
    void createSingleChipPointclouds(const float& intensityThresholdPercent);
};

}

#include "coloradar_run.hpp"

#endif
