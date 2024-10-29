#include "coloradar_tools.h"
#include <unordered_map>
#include <H5Cpp.h>

namespace fs = std::filesystem;


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

    // Create the dataset in the HDF5 file
    H5::DataSet dataset = file.createDataSet(datasetName, H5::PredType::NATIVE_FLOAT, dataspace);

    // Reserve space for the data to write to HDF5
    std::vector<float> cloudData;
    cloudData.reserve(cloud.size() * 5);  // 5 fields: x, y, z, intensity, doppler

    // Populate the data vector
    for (const auto& point : cloud) {
        cloudData.push_back(point.x);
        cloudData.push_back(point.y);
        cloudData.push_back(point.z);
        cloudData.push_back(point.intensity);
        cloudData.push_back(point.doppler);
    }

    // Write the data to the HDF5 dataset
    dataset.write(cloudData.data(), H5::PredType::NATIVE_FLOAT);
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
    coloradar::ColoradarPlusDataset dataset(coloradarDirPath);
    std::vector<std::string> targetRuns;
    if (!runName.empty()) {
        targetRuns.push_back(runName);
    } else {
        targetRuns = {"ec_hallways_run0", "ec_hallways_run2"};
    }

    H5::H5File file(coloradarDirPath / "dataset.h5", H5F_ACC_TRUNC);
    for (const auto& runName : targetRuns) {
        auto run = dataset.getRun(runName);
        auto poses = run->getPoses<octomath::Pose6D>();
        auto radarPoses = run.interpolatePoses(poses, run->poseTimestamps(), run->cascadeTimestamps());

        saveVectorToHDF5("timestamps", run->radarTimestamps(), file);
        saveVectorToHDF5("poses", radarPoses, file);

        for (size_t j = 0; j < run->cascadeTimestamps().size(); ++j) {
            std::vector<float> rawHeatmap = run->getCascadeHeatmap(j);
            std::vector<float> heatmap = coloradar::clipHeatmapImage(rawHeatmap, azimuthMaxBin, elevationMaxBin, rangeMaxBin, &dataset.cascadeConfig());
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
