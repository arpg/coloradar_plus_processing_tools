#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/complex.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl_bind.h>
#include "coloradar_tools.h"


namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(pcl::PointCloud<pcl::PointXYZI>);
PYBIND11_MAKE_OPAQUE(std::vector<coloradar::RadarPoint>);


bool isNumpyArrayEmpty(const py::array_t<float>& array) {
    if (array.is_none()) {
        return true;
    }
    if (array.ndim() != 2) {
        return true;
    }
    auto buffer = array.unchecked<2>();
    if (buffer.shape(0) == 0 || buffer.shape(1) == 0) {
        return true;
    }
    for (ssize_t i = 0; i < buffer.shape(0); ++i) {
        for (ssize_t j = 0; j < buffer.shape(1); ++j) {
            if (std::isnan(buffer(i, j))) {
                return true;
            }
        }
    }
    return false;
}


py::array_t<float> poseToNumpy(const Eigen::Affine3f& pose) {
    py::array_t<float> result({7});
    auto result_buffer = result.mutable_unchecked<1>();
    result_buffer(0) = pose.translation().x();
    result_buffer(1) = pose.translation().y();
    result_buffer(2) = pose.translation().z();
    Eigen::Quaternionf quaternion(pose.rotation());
    result_buffer(3) = quaternion.x();
    result_buffer(4) = quaternion.y();
    result_buffer(5) = quaternion.z();
    result_buffer(6) = quaternion.w();
    return result;
}

Eigen::Affine3f numpyToPose(const py::array_t<float>& array) {
    if (array.ndim() != 1 || array.shape(0) != 7) {
        throw std::runtime_error("Input array must have shape (7,)");
    }
    auto array_data = array.unchecked<1>();
    Eigen::Vector3f translation(array_data(0), array_data(1), array_data(2));
    Eigen::Quaternionf rotation(array_data(6), array_data(3), array_data(4), array_data(5));
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translate(translation);
    pose.rotate(rotation);
    return pose;
}

py::array_t<float> posesToNumpy(const std::vector<Eigen::Affine3f>& poses) {
    py::array_t<float>::ShapeContainer shape({static_cast<long int>(poses.size()), 7});
    py::array_t<float> result(shape);
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < poses.size(); ++i) {
        py::array_t<float> single_pose = poseToNumpy(poses[i]);
        auto single_pose_data = single_pose.unchecked<1>();
        for (size_t j = 0; j < 7; ++j) {
            result_buffer(i, j) = single_pose_data(j);
        }
    }
    return result;
}

std::vector<Eigen::Affine3f> numpyToPoses(const py::array_t<float>& array) {
    if (array.ndim() != 2 || array.shape(1) != 7) {
        throw std::runtime_error("Input array must have shape (N, 7)");
    }
    std::vector<Eigen::Affine3f> poses;
    poses.reserve(array.shape(0));
    auto array_data = array.unchecked<2>();
    for (ssize_t i = 0; i < array.shape(0); ++i) {
        Eigen::Vector3f translation(array_data(i, 0), array_data(i, 1), array_data(i, 2));
        Eigen::Quaternionf rotation(array_data(i, 6), array_data(i, 3), array_data(i, 4), array_data(i, 5));
        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        pose.translate(translation);
        pose.rotate(rotation);
        poses.push_back(pose);
    }
    return poses;
}


py::array_t<float> radarCloudToNumpy(const pcl::PointCloud<coloradar::RadarPoint>& cloud) {
    py::array_t<float>::ShapeContainer shape({static_cast<long int>(cloud.points.size()), 5});
    py::array_t<float> result(shape);
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& point = cloud.points[i];
        result_buffer(i, 0) = point.x;
        result_buffer(i, 1) = point.y;
        result_buffer(i, 2) = point.z;
        result_buffer(i, 3) = point.intensity;
        result_buffer(i, 4) = point.doppler;
    }
    return result;
}

py::array_t<float> pointcloudToNumpy(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    py::array_t<float>::ShapeContainer shape({static_cast<long int>(cloud.points.size()), 4});
    py::array_t<float> result(shape);
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& point = cloud.points[i];
        result_buffer(i, 0) = point.x;
        result_buffer(i, 1) = point.y;
        result_buffer(i, 2) = point.z;
        result_buffer(i, 3) = point.intensity;
    }
    return result;
}

template<typename T>
py::array_t<T> vectorToNumpy(const std::vector<T>& vec) {
    typename py::array_t<T>::ShapeContainer shape({static_cast<long int>(vec.size())});
    py::array_t<T> result(shape);
    auto result_buffer = result.template mutable_unchecked<1>();
    for (size_t i = 0; i < vec.size(); ++i) {
        result_buffer(i) = vec[i];
    }
    return result;
}


PYBIND11_MODULE(coloradar_dataset_tools, m) {
    py::class_<pcl::PointXYZI>(m, "PointXYZI")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZI::x)
        .def_readwrite("y", &pcl::PointXYZI::y)
        .def_readwrite("z", &pcl::PointXYZI::z)
        .def_readwrite("intensity", &pcl::PointXYZI::intensity);

    py::class_<pcl::PointCloud<pcl::PointXYZI>>(m, "Pointcloud4d")
        .def(py::init<>())
        .def_readwrite("points", &pcl::PointCloud<pcl::PointXYZI>::points)
        .def("width", [](const pcl::PointCloud<pcl::PointXYZI>& pc) { return pc.width; })
        .def("height", [](const pcl::PointCloud<pcl::PointXYZI>& pc) { return pc.height; })
        .def("is_dense", [](const pcl::PointCloud<pcl::PointXYZI>& pc) { return pc.is_dense; });

    // RadarConfig
    py::class_<coloradar::RadarConfig, std::shared_ptr<coloradar::RadarConfig>>(m, "RadarConfig")
        .def_readonly_static("c", &coloradar::RadarConfig::c)
        .def_readonly("num_range_bins", &coloradar::RadarConfig::numRangeBins)
        .def_readonly("num_pos_range_bins", &coloradar::RadarConfig::numPosRangeBins)
        .def_readonly("num_elevation_bins", &coloradar::RadarConfig::numElevationBins)
        .def_readonly("num_azimuth_bins", &coloradar::RadarConfig::numAzimuthBins)
        .def_readonly("range_bin_width", &coloradar::RadarConfig::rangeBinWidth)
        .def_readonly("azimuth_bins", &coloradar::RadarConfig::azimuthBins)
        .def_readonly("elevation_bins", &coloradar::RadarConfig::elevationBins)
        .def_readonly("design_frequency", &coloradar::RadarConfig::designFrequency)
        .def_readonly("num_tx_antennas", &coloradar::RadarConfig::numTxAntennas)
        .def_readonly("num_rx_antennas", &coloradar::RadarConfig::numRxAntennas)
        .def_readonly("tx_centers", &coloradar::RadarConfig::txCenters)
        .def_readonly("rx_centers", &coloradar::RadarConfig::rxCenters)
        .def_readonly("num_adc_samples_per_chirp", &coloradar::RadarConfig::numAdcSamplesPerChirp)
        .def_readonly("num_chirps_per_frame", &coloradar::RadarConfig::numChirpsPerFrame)
        .def_readonly("adc_sample_frequency", &coloradar::RadarConfig::adcSampleFrequency)
        .def_readonly("start_frequency", &coloradar::RadarConfig::startFrequency)
        .def_readonly("idle_time", &coloradar::RadarConfig::idleTime)
        .def_readonly("adc_start_time", &coloradar::RadarConfig::adcStartTime)
        .def_readonly("ramp_end_time", &coloradar::RadarConfig::rampEndTime)
        .def_readonly("frequency_slope", &coloradar::RadarConfig::frequencySlope)
        .def_readonly("num_doppler_bins", &coloradar::RadarConfig::numDopplerBins)
        .def_readonly("coupling_calib_matrix", &coloradar::RadarConfig::couplingCalibMatrix)
        .def_readonly("calib_adc_sample_frequency", &coloradar::RadarConfig::calibAdcSampleFrequency)
        .def_readonly("calib_frequency_slope", &coloradar::RadarConfig::calibFrequencySlope)
        .def_readonly("frequency_calib_matrix", &coloradar::RadarConfig::frequencyCalibMatrix)
        .def_readonly("phase_calib_matrix", &coloradar::RadarConfig::phaseCalibMatrix)
        .def_readonly("num_azimuth_beams", &coloradar::RadarConfig::numAzimuthBeams)
        .def_readonly("num_elevation_beams", &coloradar::RadarConfig::numElevationBeams)
        .def_readonly("azimuth_aperture_len", &coloradar::RadarConfig::azimuthApertureLen)
        .def_readonly("elevation_aperture_len", &coloradar::RadarConfig::elevationApertureLen)
        .def_readonly("num_angles", &coloradar::RadarConfig::numAngles)
        .def_readonly("num_virtual_elements", &coloradar::RadarConfig::numVirtualElements)
        .def_readonly("virtual_array_map", &coloradar::RadarConfig::virtualArrayMap)
        .def_readonly("azimuth_angles", &coloradar::RadarConfig::azimuthAngles)
        .def_readonly("elevation_angles", &coloradar::RadarConfig::elevationAngles)
        .def_readonly("doppler_bin_width", &coloradar::RadarConfig::dopplerBinWidth)
        .def("to_json", &coloradar::RadarConfig::toJson);

    // SingleChipConfig
    py::class_<coloradar::SingleChipConfig, coloradar::RadarConfig, std::shared_ptr<coloradar::SingleChipConfig>>(m, "SingleChipConfig")
        .def(py::init<const std::filesystem::path&, const int&, const int&>(), py::arg("calib_dir"), py::arg("num_azimuth_beams") = 64, py::arg("num_elevation_beams") = 8);

    // CascadeConfig
    py::class_<coloradar::CascadeConfig, coloradar::RadarConfig, std::shared_ptr<coloradar::CascadeConfig>>(m, "CascadeConfig")
        .def(py::init<const std::filesystem::path&, const int&, const int&>(), py::arg("calib_dir"), py::arg("num_azimuth_beams") = 128, py::arg("num_elevation_beams") = 32);

    // ColoradarPlusRun
    py::class_<coloradar::ColoradarPlusRun>(m, "ColoradarPlusRun")
        .def(py::init<const std::filesystem::path&, coloradar::RadarConfig*>())
        .def("pose_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.poseTimestamps()); })
        .def("imu_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.imuTimestamps()); })
        .def("lidar_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.lidarTimestamps()); })
        .def("cascade_cube_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.cascadeCubeTimestamps()); })
        .def("cascade_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.cascadeTimestamps()); })

        .def("get_lidar_pointcloud", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>))
        .def("get_lidar_pointcloud", py::overload_cast<const int&>(&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>))
        .def("get_cascade_datacube", [](coloradar::ColoradarPlusRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getCascadeDatacube(binFilePath)); })
        .def("get_cascade_datacube", [](coloradar::ColoradarPlusRun& self, const int& cubeIdx) { return vectorToNumpy(self.getCascadeDatacube(cubeIdx)); })
        .def("get_cascade_heatmap", [](coloradar::ColoradarPlusRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getCascadeHeatmap(binFilePath)); })
        .def("get_cascade_heatmap", [](coloradar::ColoradarPlusRun& self, const int& hmIdx) { return vectorToNumpy(self.getCascadeHeatmap(hmIdx)); })
        .def("create_cascade_pointclouds", &coloradar::ColoradarPlusRun::createCascadePointclouds, py::arg("intensity_threshold_percent") = 0)
        .def("get_cascade_pointcloud", [](coloradar::ColoradarPlusRun& self, std::filesystem::path binFilePath, float intensityThresholdPercent = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint> cloud = self.getCascadePointcloud(binFilePath, intensityThresholdPercent); return radarCloudToNumpy(cloud);
        }, py::arg("bin_file_path"), py::arg("intensity_threshold_percent") = 0)
        .def("get_cascade_pointcloud", [](coloradar::ColoradarPlusRun& self, int cloudIdx, float intensityThresholdPercent = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint> cloud = self.getCascadePointcloud(cloudIdx, intensityThresholdPercent); return radarCloudToNumpy(cloud);
        }, py::arg("cloud_idx"), py::arg("intensity_threshold_percent") = 0)

        .def("create_lidar_octomap", [](coloradar::ColoradarPlusRun& self, const double mapResolution, const float lidarTotalHorizontalFov, const float lidarTotalVerticalFov, const float lidarMaxRange, const py::array_t<float>& baseToLidarTransformArray) {
            self.createLidarOctomap(mapResolution, lidarTotalHorizontalFov, lidarTotalVerticalFov, lidarMaxRange, numpyToPose(baseToLidarTransformArray));
        }, py::arg("map_resolution") = 0.5, py::arg("lidar_total_horizontal_fov") = 360, py::arg("lidar_total_vertical_fov") = 180, py::arg("lidar_max_range") = 100, py::arg("base_to_lidar_transform") = poseToNumpy(Eigen::Affine3f::Identity()))
        .def("sample_map_frames", [](coloradar::ColoradarPlusRun& self, const float& totalHorizontalFov, const float& totalVerticalFov, const float& range, const py::array_t<float>& baseToSensorTransformArray, const py::array_t<float>& basePosesArray) {
            self.sampleMapFrames(totalHorizontalFov, totalVerticalFov, range, numpyToPose(baseToSensorTransformArray), numpyToPoses(basePosesArray));
        }, py::arg("total_horizontal_fov") = 360, py::arg("total_vertical_fov") = 180, py::arg("range") = 100, py::arg("base_to_sensor_transform") = poseToNumpy(Eigen::Affine3f::Identity()), py::arg("base_poses") = py::none())
        .def("get_lidar_octomap", [](coloradar::ColoradarPlusRun& self) { auto octree = self.readLidarOctomap(); return pointcloudToNumpy(octree); })
        .def("get_map_frame", [](coloradar::ColoradarPlusRun& self, const int& frameIdx) { return pointcloudToNumpy(self.readMapFrame(frameIdx)); }, py::arg("frame_idx"))

        .def("get_poses", [](coloradar::ColoradarPlusRun& self) {
            std::vector<Eigen::Affine3f> poses = self.getPoses<Eigen::Affine3f>(); return posesToNumpy(poses);
        }, "Returns poses as an Nx7 numpy array [x, y, z, qx, qy, qz, qw]")
        .def("interpolate_poses", [](coloradar::ColoradarPlusRun& self, const py::array_t<float>& poses_array, const std::vector<double>& pose_timestamps, const std::vector<double>& target_timestamps) {
            std::vector<Eigen::Affine3f> interpolated_poses = self.interpolatePoses<Eigen::Affine3f>(numpyToPoses(poses_array), pose_timestamps, target_timestamps); return posesToNumpy(interpolated_poses);
        }, py::arg("poses"), py::arg("pose_timestamps"), py::arg("target_timestamps"), "Interpolates poses and returns an Nx7 numpy array [x, y, z, qx, qy, qz, qw]");

    // ColoradarRun
    py::class_<coloradar::ColoradarRun, coloradar::ColoradarPlusRun>(m, "ColoradarRun")
        .def(py::init<const std::filesystem::path&, coloradar::RadarConfig*, coloradar::RadarConfig*>())
        .def("single_chip_cube_timestamps", [](coloradar::ColoradarRun& self) { return vectorToNumpy(self.singleChipCubeTimestamps()); })
        .def("single_chip_timestamps", [](coloradar::ColoradarRun& self) { return vectorToNumpy(self.singleChipTimestamps()); })
        .def("get_single_chip_datacube", [](coloradar::ColoradarRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getSingleChipDatacube(binFilePath)); })
        .def("get_single_chip_datacube", [](coloradar::ColoradarRun& self, const int& cubeIdx) { return vectorToNumpy(self.getSingleChipDatacube(cubeIdx)); })
        .def("get_single_chip_heatmap", [](coloradar::ColoradarRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getSingleChipHeatmap(binFilePath)); })
        .def("get_single_chip_heatmap", [](coloradar::ColoradarRun& self, const int& hmIdx) { return vectorToNumpy(self.getSingleChipHeatmap(hmIdx)); })
        .def("get_single_chip_pointcloud", [](coloradar::ColoradarRun& self, std::filesystem::path binFilePath, float intensityThresholdPercent = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint> cloud = self.getSingleChipPointcloud(binFilePath, intensityThresholdPercent); return radarCloudToNumpy(cloud);
        }, py::arg("bin_file_path"), py::arg("intensity_threshold_percent") = 0)

        .def("get_single_chip_pointcloud", [](coloradar::ColoradarRun& self, int cloudIdx, float intensityThresholdPercent = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint> cloud = self.getSingleChipPointcloud(cloudIdx, intensityThresholdPercent); return radarCloudToNumpy(cloud);
        }, py::arg("cloud_idx"), py::arg("intensity_threshold_percent") = 0);

    // ColoradarPlusDataset
    py::class_<coloradar::ColoradarPlusDataset>(m, "ColoradarPlusDataset")
        .def(py::init<const std::filesystem::path&>())
        .def("list_runs", &coloradar::ColoradarPlusDataset::listRuns)
        .def("get_runs", &coloradar::ColoradarPlusDataset::getRuns, py::return_value_policy::reference)
        .def("get_run", &coloradar::ColoradarPlusDataset::getRun, py::return_value_policy::reference)
        .def("imu_transform", [](coloradar::ColoradarPlusDataset& self) { return poseToNumpy(self.imuTransform()); })
        .def("lidar_transform", [](coloradar::ColoradarPlusDataset& self) { return poseToNumpy(self.lidarTransform()); })
        .def("cascade_transform", [](coloradar::ColoradarPlusDataset& self) { return poseToNumpy(self.cascadeTransform()); })
        .def("cascade_config", &coloradar::ColoradarPlusDataset::cascadeConfig, py::return_value_policy::reference)
        .def("export_to_file", [](
            coloradar::ColoradarPlusDataset self,
            std::vector<coloradar::ColoradarPlusRun*> runs = {},
            const std::string& destination = "",
            const bool& includeCascadeHeatmaps = false,
            const bool& includeCascadePointclouds = false,
            const bool& cascadePointcloudsInGlobalFrame = false,
            const int& cascadeAzimuthMaxBin = -1,
            const int& cascadeElevationMaxBin = -1,
            const int& cascadeRangeMaxBin = -1,
            const bool& removeCascadeDopplerDim = false,
            const bool& collapseCascadeElevation = false,
            const int& collapseCascadeElevationMinZ = -100,
            const int& collapseCascadeElevationMaxZ = 100,
            const float& cascadeCloudIntensityThresholdPercent = 0.0f,
            const bool& includeLidarFrames = false,
            const float& lidarFrameTotalHorizontalFov = 360.0f,
            const float& lidarFrameTotalVerticalFov = 180.0f,
            const float& lidarFrameMaxRange = 100.0f,
            const bool& collapseLidarFrameElevation = false,
            const float& collapseLidarFrameElevationMinZ = -100.0f,
            const float& collapseLidarFrameElevationMaxZ = 100.0f,
            const bool& includeLidarMap = false,
            const bool& collapseMapElevation = false,
            const float& collapseMapElevationMinZ = -100.0f,
            const float& collapseMapElevationMaxZ = 100.0f,
            const py::array_t<float>& lidarMapTransform = poseToNumpy(Eigen::Affine3f::Identity()),
            const bool& includeMapFrames = false,
            const float& mapSampleTotalHorizontalFov = 360.0f,
            const float& mapSampleTotalVerticalFov = 180.0f,
            const float& mapSampleMaxRange = 100.0f,
            const py::array_t<float>& mapSamplingBaseToSensorTransformArray = poseToNumpy(Eigen::Affine3f::Identity()),
            const py::array_t<float>& mapSamplingBasePosesArray = py::none(),
            const bool& collapseMapSampleElevation = false,
            const float& collapseMapSampleElevationMinZ = -100.0f,
            const float& collapseMapSampleElevationMaxZ = 100.0f,
            const bool& removeLidarIntensity = false,
            const bool& includeTruePoses = true,
            const bool& includeCascadePoses = true,
            const bool& includeLidarPoses = true,
            const bool& includeTrueTimestamps = true,
            const bool& includeCascadeTimestamps = true,
            const bool& includeLidarTimestamps = true
        ) {
            auto lidarMapTransformEig = numpyToPose(lidarMapTransform);
            auto mapSamplingBaseToSensorTransform = numpyToPose(mapSamplingBaseToSensorTransformArray);
            auto mapSamplingBasePoses = isNumpyArrayEmpty(mapSamplingBasePosesArray) ? std::vector<Eigen::Affine3f>{} : numpyToPoses(mapSamplingBasePosesArray);
            return self.exportToFile(
                runs, destination,
                includeCascadeHeatmaps, includeCascadePointclouds, cascadePointcloudsInGlobalFrame, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, removeCascadeDopplerDim, collapseCascadeElevation, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, cascadeCloudIntensityThresholdPercent,
                includeLidarFrames, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange, collapseLidarFrameElevation, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ,
                includeLidarMap, collapseMapElevation, collapseMapElevationMinZ, collapseMapElevationMaxZ, lidarMapTransformEig,
                includeMapFrames, mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingBaseToSensorTransform, mapSamplingBasePoses, collapseMapSampleElevation, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ,
                removeLidarIntensity,
                includeTruePoses, includeCascadePoses, includeLidarPoses,
                includeTrueTimestamps, includeCascadeTimestamps, includeLidarTimestamps
            );
        },
            py::arg("runs") = std::vector<coloradar::ColoradarPlusRun*>{},
            py::arg("destination") = "",
            py::arg("include_cascade_heatmaps") = false,
            py::arg("include_cascade_pointclouds") = false,
            py::arg("cascade_pointclouds_in_global_frame") = false,
            py::arg("cascade_azimuth_max_bin") = -1,
            py::arg("cascade_elevation_max_bin") = -1,
            py::arg("cascade_range_max_bin") = -1,
            py::arg("remove_cascade_doppler_dim") = false,
            py::arg("collapse_cascade_elevation") = false,
            py::arg("collapse_cascade_elevation_min_z") = -100,
            py::arg("collapse_cascade_elevation_max_z") = 100,
            py::arg("cascade_cloud_intensity_threshold_percent") = 0.0f,
            py::arg("include_lidar_frames") = false,
            py::arg("lidar_frame_total_horizontal_fov") = 360.0f,
            py::arg("lidar_frame_total_vertical_fov") = 180.0f,
            py::arg("lidar_frame_max_range") = 100.0f,
            py::arg("collapse_lidar_frame_elevation") = false,
            py::arg("collapse_lidar_frame_elevation_min_z") = -100.0f,
            py::arg("collapse_lidar_frame_elevation_max_z") = 100.0f,
            py::arg("include_lidar_map") = false,
            py::arg("collapse_map_elevation") = false,
            py::arg("collapse_map_elevation_min_z") = -100.0f,
            py::arg("collapse_map_elevation_max_z") = 100.0f,
            py::arg("lidar_map_transform") = poseToNumpy(Eigen::Affine3f::Identity()),
            py::arg("include_map_frames") = false,
            py::arg("map_sample_total_horizontal_fov") = 360.0f,
            py::arg("map_sample_total_vertical_fov") = 180.0f,
            py::arg("map_sample_max_range") = 100.0f,
            py::arg("map_sampling_base_to_sensor_transform") = poseToNumpy(Eigen::Affine3f::Identity()),
            py::arg("map_sampling_base_poses") = py::none(),
            py::arg("collapse_map_sample_elevation") = false,
            py::arg("collapse_map_sample_elevation_min_z") = -100.0f,
            py::arg("collapse_map_sample_elevation_max_z") = 100.0f,
            py::arg("remove_lidar_intensity") = false,
            py::arg("include_true_poses") = true,
            py::arg("include_cascade_poses") = true,
            py::arg("include_lidar_poses") = true,
            py::arg("include_true_timestamps") = true,
            py::arg("include_cascade_timestamps") = true,
            py::arg("include_lidar_timestamps") = true
    );

    // ColoradarDataset
    py::class_<coloradar::ColoradarDataset, coloradar::ColoradarPlusDataset>(m, "ColoradarDataset")
        .def(py::init<const std::filesystem::path&>())
        .def("get_run", &coloradar::ColoradarDataset::getRun, py::return_value_policy::reference)
        .def("single_chip_transform", [](coloradar::ColoradarDataset& self) { return poseToNumpy(self.singleChipTransform()); })
        .def("single_chip_config", &coloradar::ColoradarDataset::singleChipConfig, py::return_value_policy::reference);
}
