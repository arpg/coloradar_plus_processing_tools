#ifndef UTILS_H
#define UTILS_H

#include "configs.h"

#include <filesystem>
#include <string>
#include <vector>
#include <Eigen/Dense>


namespace coloradar {

struct RadarPoint
{
  PCL_ADD_POINT4D;
  float intensity;
  float doppler;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


template <Pcl4dPointType PointT, template <PclCloudType> class CloudT> void octreeToPcl(const octomap::OcTree& tree, CloudT<PointT>& cloud);
template <PclPointType PointT, template <PclCloudType> class CloudT> void filterFov(CloudT<PointT>& cloud, const float& horizontalFov, const float& verticalFov, const float& range);
pcl::PointCloud<RadarPoint> heatmapToPointcloud(const std::vector<float>& heatmap, coloradar::RadarConfig* config, const float& intensityThresholdPercent = 0.0);
std::vector<float> clipHeatmapImage(const std::vector<float>& image, const float& horizontalFov, const float& verticalFov, const float& range, const coloradar::RadarConfig* config);
std::vector<float> clipHeatmapImage(const std::vector<float>& image, const int& azimuthMaxBin, const int& elevationMaxBin, const int& rangeMaxBin, const coloradar::RadarConfig* config);


class OctoPointcloud : public octomap::Pointcloud {
public:
    OctoPointcloud() = default;
    OctoPointcloud(const OctoPointcloud& other) : octomap::Pointcloud(other) {}
    template <PclPointType PointT, template <PclCloudType> class CloudT> OctoPointcloud(const CloudT<PointT>& cloud);

    template <PclCloudType CloudT> CloudT toPcl();

    void filterFov(const float& horizontalFovTan, const float& verticalFovTan, const float& range);
    void transform(const Eigen::Affine3f& transformMatrix);
    using octomap::Pointcloud::transform;
};

}

#include "pcl_functions.hpp"
#include "octo_pointcloud.hpp"

#endif
