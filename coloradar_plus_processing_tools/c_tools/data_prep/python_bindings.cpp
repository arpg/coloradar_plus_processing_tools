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


PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Affine3f>);
PYBIND11_MAKE_OPAQUE(pcl::PointCloud<pcl::PointXYZI>);
PYBIND11_MAKE_OPAQUE(std::vector<coloradar::RadarPoint>);


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

py::array_t<float> posesToNumpy(const std::vector<Eigen::Affine3f>& poses) {
    py::array_t<float>::ShapeContainer shape({static_cast<long int>(poses.size()), 7});
    py::array_t<float> result(shape);
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& pose = poses[i];
        result_buffer(i, 0) = pose.translation().x();
        result_buffer(i, 1) = pose.translation().y();
        result_buffer(i, 2) = pose.translation().z();
        Eigen::Quaternionf quaternion(pose.rotation());
        result_buffer(i, 3) = quaternion.x();
        result_buffer(i, 4) = quaternion.y();
        result_buffer(i, 5) = quaternion.z();
        result_buffer(i, 6) = quaternion.w();
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
    // Helper bindings
    py::class_<Eigen::Affine3f>(m, "Affine3f")
        .def(py::init<>())
        .def("translation", [](const Eigen::Affine3f& self) -> Eigen::Vector3f { return self.translation(); })
        .def("rotation", [](const Eigen::Affine3f& self) -> Eigen::Quaternionf { return Eigen::Quaternionf(self.rotation()); });
    py::bind_vector<std::vector<Eigen::Affine3f>>(m, "Affine3fVector");

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
        .def("create_lidar_octomap", &coloradar::ColoradarPlusRun::createLidarOctomap, py::arg("map_resolution"), py::arg("lidar_total_horizontal_fov"), py::arg("lidar_total_vertical_fov"), py::arg("lidar_max_range"), py::arg("lidar_transform") = Eigen::Affine3f::Identity())
        .def("get_lidar_octomap", [](coloradar::ColoradarPlusRun& self) { auto octree = self.readLidarOctomap(); return pointcloudToNumpy(octree); })
        .def("sample_map_frames", &coloradar::ColoradarPlusRun::sampleMapFrames)
        .def("read_map_frame", &coloradar::ColoradarPlusRun::readMapFrame)
        .def("get_poses", [](coloradar::ColoradarPlusRun& self) {
            std::vector<Eigen::Affine3f> poses = self.getPoses<Eigen::Affine3f>(); return posesToNumpy(poses);
        }, "Returns poses as an Nx7 numpy array [x, y, z, qx, qy, qz, qw]")
        .def("interpolate_poses", [](coloradar::ColoradarPlusRun& self, const py::array_t<float>& poses_array, const std::vector<double>& pose_timestamps, const std::vector<double>& target_timestamps) {
            std::vector<Eigen::Affine3f> interpolated_poses = self.interpolatePoses<Eigen::Affine3f>(numpyToPoses(poses_array), pose_timestamps, target_timestamps); return posesToNumpy(interpolated_poses);
        }, py::arg("poses"), py::arg("pose_timestamps"), py::arg("target_timestamps"), "Interpolates poses and returns an Nx7 numpy array [x, y, z, qx, qy, qz, qw]")
        .def("get_lidar_pointcloud", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>))
        .def("get_lidar_pointcloud", py::overload_cast<const int&>(&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>));

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
        .def("imu_transform", &coloradar::ColoradarPlusDataset::imuTransform)
        .def("lidar_transform", &coloradar::ColoradarPlusDataset::lidarTransform)
        .def("cascade_transform", &coloradar::ColoradarPlusDataset::cascadeTransform)
        .def("cascade_config", &coloradar::ColoradarPlusDataset::cascadeConfig, py::return_value_policy::reference);

    // ColoradarDataset
    py::class_<coloradar::ColoradarDataset, coloradar::ColoradarPlusDataset>(m, "ColoradarDataset")
        .def(py::init<const std::filesystem::path&>())
        .def("get_run", &coloradar::ColoradarDataset::getRun, py::return_value_policy::reference)
        .def("single_chip_transform", &coloradar::ColoradarDataset::singleChipTransform)
        .def("single_chip_config", &coloradar::ColoradarDataset::singleChipConfig, py::return_value_policy::reference);
}
