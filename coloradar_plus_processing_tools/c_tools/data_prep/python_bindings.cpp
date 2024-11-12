#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/complex.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include "coloradar_tools.h"


namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Affine3f>);
PYBIND11_MAKE_OPAQUE(pcl::PointCloud<pcl::PointXYZI>);
PYBIND11_MAKE_OPAQUE(std::vector<coloradar::RadarPoint>);


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

    py::class_<coloradar::RadarPoint>(m, "RadarPoint")
        .def(py::init<>())
        .def_readwrite("x", &coloradar::RadarPoint::x)
        .def_readwrite("y", &coloradar::RadarPoint::y)
        .def_readwrite("z", &coloradar::RadarPoint::z)
        .def_readwrite("intensity", &coloradar::RadarPoint::intensity)
        .def_readwrite("doppler", &coloradar::RadarPoint::doppler);

    py::class_<pcl::PointCloud<coloradar::RadarPoint>>(m, "RadarPointcloud")
        .def(py::init<>())
        .def_readwrite("points", &pcl::PointCloud<coloradar::RadarPoint>::points)
        .def("width", [](const pcl::PointCloud<coloradar::RadarPoint>& pc) { return pc.width; })
        .def("height", [](const pcl::PointCloud<coloradar::RadarPoint>& pc) { return pc.height; })
        .def("is_dense", [](const pcl::PointCloud<coloradar::RadarPoint>& pc) { return pc.is_dense; });

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
        .def("pose_timestamps", &coloradar::ColoradarPlusRun::poseTimestamps, py::return_value_policy::reference_internal)
        .def("imu_timestamps", &coloradar::ColoradarPlusRun::imuTimestamps, py::return_value_policy::reference_internal)
        .def("lidar_timestamps", &coloradar::ColoradarPlusRun::lidarTimestamps, py::return_value_policy::reference_internal)
        .def("cascade_cube_timestamps", &coloradar::ColoradarPlusRun::cascadeCubeTimestamps, py::return_value_policy::reference_internal)
        .def("cascade_timestamps", &coloradar::ColoradarPlusRun::cascadeTimestamps, py::return_value_policy::reference_internal)
        .def("get_cascade_datacube", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarPlusRun::getCascadeDatacube))
        .def("get_cascade_datacube", py::overload_cast<const int&>(&coloradar::ColoradarPlusRun::getCascadeDatacube))
        .def("get_cascade_heatmap", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarPlusRun::getCascadeHeatmap))
        .def("get_cascade_heatmap", py::overload_cast<const int&>(&coloradar::ColoradarPlusRun::getCascadeHeatmap))
        .def("create_cascade_pointclouds", &coloradar::ColoradarPlusRun::createCascadePointclouds, py::arg("intensity_threshold_percent") = 0)
        .def("get_cascade_pointcloud", py::overload_cast<const std::filesystem::path&, const float&>(&coloradar::ColoradarPlusRun::getCascadePointcloud), py::arg("bin_file_path"), py::arg("intensity_threshold_percent") = 0)
        .def("get_cascade_pointcloud", py::overload_cast<const int&, const float&>(&coloradar::ColoradarPlusRun::getCascadePointcloud), py::arg("cloud_idx"), py::arg("intensity_threshold_percent") = 0)
        .def("build_lidar_octomap", &coloradar::ColoradarPlusRun::buildLidarOctomap, py::arg("map_resolution"), py::arg("lidar_total_horizontal_fov"), py::arg("lidar_total_vertical_fov"), py::arg("lidar_max_range"), py::arg("lidar_transform") = Eigen::Affine3f::Identity())
        .def("save_lidar_octomap", &coloradar::ColoradarPlusRun::saveLidarOctomap)
        .def("read_lidar_octomap", &coloradar::ColoradarPlusRun::readLidarOctomap)
        .def("sample_map_frames", &coloradar::ColoradarPlusRun::sampleMapFrames)
        .def("read_map_frame", &coloradar::ColoradarPlusRun::readMapFrame)
        .def("get_poses", &coloradar::ColoradarPlusRun::getPoses<Eigen::Affine3f>, "Returns a list of poses as Affine3fVector")
        .def("interpolate_poses", &coloradar::ColoradarPlusRun::interpolatePoses<Eigen::Affine3f>, py::arg("poses"), py::arg("pose_timestamps"), py::arg("target_timestamps"))
        .def("get_lidar_pointcloud", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>))
        .def("get_lidar_pointcloud", py::overload_cast<const int&>(&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>));

    // ColoradarRun (Ensure inheritance)
    py::class_<coloradar::ColoradarRun, coloradar::ColoradarPlusRun>(m, "ColoradarRun")
        .def(py::init<const std::filesystem::path&, coloradar::RadarConfig*, coloradar::RadarConfig*>())
        .def("single_chip_cube_timestamps", &coloradar::ColoradarRun::singleChipCubeTimestamps, py::return_value_policy::reference_internal)
        .def("single_chip_timestamps", &coloradar::ColoradarRun::singleChipTimestamps, py::return_value_policy::reference_internal)
        .def("get_single_chip_datacube", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarRun::getSingleChipDatacube))
        .def("get_single_chip_datacube", py::overload_cast<const int&>(&coloradar::ColoradarRun::getSingleChipDatacube))
        .def("get_single_chip_heatmap", py::overload_cast<const std::filesystem::path&>(&coloradar::ColoradarRun::getSingleChipHeatmap))
        .def("get_single_chip_heatmap", py::overload_cast<const int&>(&coloradar::ColoradarRun::getSingleChipHeatmap))
        .def("get_single_chip_pointcloud", py::overload_cast<const std::filesystem::path&, const float&>(&coloradar::ColoradarRun::getSingleChipPointcloud),  py::arg("bin_file_path"), py::arg("intensity_threshold_percent") = 0)
        .def("get_single_chip_pointcloud", py::overload_cast<const int&, const float&>(&coloradar::ColoradarRun::getSingleChipPointcloud), py::arg("cloud_idx"), py::arg("intensity_threshold_percent") = 0);
        // .def("create_single_chip_pointclouds", &coloradar::ColoradarRun::createSingleChipPointclouds, py::arg("intensity_threshold_percent") = 0);

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

    // ColoradarDataset (Ensure inheritance)
    py::class_<coloradar::ColoradarDataset, coloradar::ColoradarPlusDataset>(m, "ColoradarDataset")
        .def(py::init<const std::filesystem::path&>())
        .def("get_run", &coloradar::ColoradarDataset::getRun, py::return_value_policy::reference)
        .def("single_chip_transform", &coloradar::ColoradarDataset::singleChipTransform)
        .def("single_chip_config", &coloradar::ColoradarDataset::singleChipConfig, py::return_value_policy::reference);
}
