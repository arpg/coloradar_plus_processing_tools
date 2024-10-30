#include "coloradar_tools.h"
#include <unordered_map>
#include <H5Cpp.h>

namespace fs = std::filesystem;


#include <sstream>
#include <iomanip>
#include <json/json.h>  // Assuming you have a JSON library, like nlohmann/json or similar


std::unordered_map<std::string, std::string> parseArguments(int argc, char** argv) {
    std::unordered_map<std::string, std::string> arguments;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find("=") != std::string::npos) {
            auto pos = arg.find("=");
            std::string key = arg.substr(0, pos);
            std::string value = arg.substr(pos + 1);
            arguments[key] = value;
        }
    }
    return arguments;
}


void saveVectorToHDF5(const std::string& datasetName, const std::vector<double>& data, H5::H5File& file) {
    hsize_t dims[1] = { data.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_DOUBLE, dataspace);
    dataset.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

void saveHeatmapToHDF5(const std::string& datasetName, const std::vector<float>& heatmap, H5::H5File& file) {
    hsize_t dims[1] = { heatmap.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_FLOAT, dataspace);
    dataset.write(heatmap.data(), H5::PredType::NATIVE_FLOAT);
}

void savePointCloudToHDF5(const std::string& datasetName, const pcl::PointCloud<pcl::PointXYZI>& cloud, H5::H5File& file) {
    hsize_t dims[2] = { cloud.size(), 4 }; // XYZ + intensity
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_FLOAT, dataspace);

    std::vector<float> cloudData;
    cloudData.reserve(cloud.size() * 4); // Reserve space for XYZ + intensity
    for (const auto& point : cloud) {
        cloudData.push_back(point.x);
        cloudData.push_back(point.y);
        cloudData.push_back(point.z);
        cloudData.push_back(point.intensity);
    }
    dataset.write(cloudData.data(), H5::PredType::NATIVE_FLOAT);
}

void saveRadarPointCloudToHDF5(const std::string& datasetName, const pcl::PointCloud<coloradar::RadarPoint>& cloud, H5::H5File& file) {
    hsize_t dims[2] = { cloud.size(), 5 };  // XYZ + intensity + doppler
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_FLOAT, dataspace);
    std::vector<float> cloudData;
    cloudData.reserve(cloud.size() * 5);

    for (const auto& point : cloud) {
        cloudData.push_back(point.x);
        cloudData.push_back(point.y);
        cloudData.push_back(point.z);
        cloudData.push_back(point.intensity);
        cloudData.push_back(point.doppler);
    }
    dataset.write(cloudData.data(), H5::PredType::NATIVE_FLOAT);
}

void saveRadarPosesToHDF5(const std::string& datasetName, const std::vector<octomath::Pose6D>& poses, H5::H5File& file) {
    hsize_t dims[2] = { poses.size(), 7 };  // Each pose has 7 values: x, y, z, qx, qy, qz, qw
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_DOUBLE, dataspace);
    std::vector<double> poseData;
    poseData.reserve(poses.size() * 7);

    for (const auto& pose : poses) {
        poseData.push_back(pose.x());
        poseData.push_back(pose.y());
        poseData.push_back(pose.z());
        const auto& quat = pose.rot();
        poseData.push_back(quat.x());
        poseData.push_back(quat.y());
        poseData.push_back(quat.z());
        poseData.push_back(quat.u());
    }
    dataset.write(poseData.data(), H5::PredType::NATIVE_DOUBLE);
}

void saveJsonToHDF5(const std::string& datasetName, const std::string& jsonStr, H5::H5File& file) {
    hsize_t dims[1] = { jsonStr.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_CHAR, dataspace);
    dataset.write(jsonStr.c_str(), H5::PredType::NATIVE_CHAR);
}


int main(int argc, char** argv) {
    auto args = parseArguments(argc, argv);
    std::string coloradarDir = (args.find("coloradarDir") != args.end()) ? args["coloradarDir"] : (argc > 1 ? argv[1] : "");
    std::string runName = (args.find("runName") != args.end()) ? args["runName"] : (argc > 2 ? argv[2] : "");
    if (coloradarDir.empty()) {
        std::cerr << "Usage: " << argv[0] << " <coloradarDir> [<runName>] [azimuthMaxBin=<idx>] [elevationMaxBin=<idx>] [rangeMaxBin=<idx>]" << std::endl;
        return -1;
    }
    int azimuthMaxBin = args.find("azimuthMaxBin") != args.end() ? std::stoi(args["azimuthMaxBin"]) : 0;
    int elevationMaxBin = args.find("elevationMaxBin") != args.end() ? std::stoi(args["elevationMaxBin"]) : 0;
    int rangeMaxBin = args.find("rangeMaxBin") != args.end() ? std::stoi(args["rangeMaxBin"]) : 0;

    fs::path coloradarDirPath(coloradarDir);
    coloradar::ColoradarDataset dataset(coloradarDirPath);
    std::vector<std::string> targetRuns;
    if (!runName.empty()) {
        targetRuns.push_back(runName);
    } else {
        targetRuns = {"ec_hallways_run0", "ec_hallways_run2"};
    }

    H5::H5File file(coloradarDirPath / "dataset.h5", H5F_ACC_TRUNC);
    auto config = dataset.cascadeConfig();
    saveJsonToHDF5("radar_config", config->toJson(), file);

    for (const auto& runName : targetRuns) {
        auto run = dataset.getRun(runName);

        auto poses = run->getPoses<octomath::Pose6D>();
        auto radarPoses = run->interpolatePoses(poses, run->poseTimestamps(), run->cascadeTimestamps());
        saveVectorToHDF5("timestamps", run->cascadeTimestamps(), file);
        saveRadarPosesToHDF5("poses", radarPoses, file);

        auto lidarMap = run->readLidarOctomap();
        savePointCloudToHDF5("lidar_map", lidarMap, file);

        for (size_t j = 0; j < run->cascadeTimestamps().size(); ++j) {
            std::vector<float> rawHeatmap = run->getCascadeHeatmap(j);
            std::vector<float> heatmap = coloradar::clipHeatmapImage(rawHeatmap, azimuthMaxBin, elevationMaxBin, rangeMaxBin, config);
            saveHeatmapToHDF5("radar_heatmap_" + std::to_string(j), heatmap, file);

            auto cloud = run->getCascadePointcloud(j, 0);
            saveRadarPointCloudToHDF5("radar_cloud_" + std::to_string(j), cloud, file);

            pcl::PointCloud<pcl::PointXYZI> mapFrame = run->readMapFrame(j);
            savePointCloudToHDF5("lidar_map_frame_" + std::to_string(j), mapFrame, file);
        }
    }
    file.close();
    return 0;
}
