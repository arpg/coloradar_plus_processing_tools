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
