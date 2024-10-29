#include "utils.h"

#include <unordered_map>
#include <tuple>
#include <functional>


POINT_CLOUD_REGISTER_POINT_STRUCT (coloradar::RadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, doppler, doppler))


template <>
struct std::hash<std::tuple<float, float, float>> {
    std::size_t operator()(const std::tuple<float, float, float>& key) const {
        auto roundToPrecision = [](float value, float precision) {
            return std::round(value / precision) * precision;
        };
        std::size_t h1 = std::hash<float>{}(roundToPrecision(std::get<0>(key), 0.01f));
        std::size_t h2 = std::hash<float>{}(roundToPrecision(std::get<1>(key), 0.01f));
        std::size_t h3 = std::hash<float>{}(roundToPrecision(std::get<2>(key), 0.01f));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
inline std::tuple<float, float, float> makeKey(float x, float y, float z) {
    return std::make_tuple(x, y, z);
}

pcl::PointCloud<coloradar::RadarPoint> coloradar::heatmapToPointcloud(const std::vector<float>& heatmap, coloradar::RadarConfig* config, const float& intensityThresholdPercent) {
    if (intensityThresholdPercent < 0 or intensityThresholdPercent >= 100)
        throw std::runtime_error("Invalid intensityThresholdPercent: expected value in [0; 100), got " + std::to_string(intensityThresholdPercent));
    float maxIntensity = 0;
    pcl::PointCloud<coloradar::RadarPoint> cloud;
    std::unordered_map<std::tuple<float, float, float>, RadarPoint, std::hash<std::tuple<float, float, float>>> pointMap;

    for (int azIdx = 0; azIdx < config->numAzimuthBeams; azIdx++) {
        for (int rangeIdx = 10; rangeIdx < config->numPosRangeBins; rangeIdx++) {
            float maxElIntensity = 0.0f;
            float maxElDoppler = 0.0f;
            int maxElBin = 0;

            for (int elIdx = 0; elIdx < config->numElevationBeams; elIdx++) {
                int angleIdx = azIdx + config->numAzimuthBeams * elIdx;
                int outIdx = 2 * (rangeIdx + config->numPosRangeBins * angleIdx);
                if (heatmap[outIdx] > maxElIntensity) {
                    maxElIntensity = heatmap[outIdx];
                    maxElDoppler = heatmap[outIdx + 1];
                    maxElBin = elIdx;
                }
            }
            if (maxElIntensity > maxIntensity)
                maxIntensity = maxElIntensity;
            double range = rangeIdx * config->rangeBinWidth;
            Eigen::Vector3f location = coloradar::internal::sphericalToCartesian(config->azimuthAngles[azIdx], config->elevationAngles[maxElBin], range);
            RadarPoint point;
            point.x = location.x();
            point.y = location.y();
            point.z = location.z();
            point.intensity = maxElIntensity;
            point.doppler = maxElDoppler;
            auto key = makeKey(point.x, point.y, point.z);
            auto it = pointMap.find(key);
            if (it != pointMap.end()) {
                if (it->second.intensity < point.intensity) {
                    it->second = point;
                }
            } else {
                pointMap[key] = point;
            }
        }
    }
    float intensityThreshold = maxIntensity * intensityThresholdPercent / 100;
    for (const auto& kv : pointMap) {
        if (kv.second.intensity >= intensityThreshold)
            cloud.push_back(kv.second);
    }
    return cloud;
}

std::vector<float> coloradar::clipHeatmapImage(const std::vector<float>& image, const float& horizontalFov, const float& verticalFov, const float& range, coloradar::RadarConfig* config) {
    if (horizontalFov <= 0 || horizontalFov > 360) {
        throw std::runtime_error("Invalid horizontal FOV value: expected 0 < FOV <= 360, got " + std::to_string(horizontalFov));
    }
    if (verticalFov <= 0 || verticalFov > 180) {
        throw std::runtime_error("Invalid vertical FOV value: expected 0 < FOV <= 180, got " + std::to_string(verticalFov));
    }
    if (range <= 0) {
        throw std::runtime_error("Invalid max range value: expected R > 0, got " + std::to_string(range));
    }
    float horizontalHalfFovRad = horizontalFov / 2 * M_PI / 180.0f;
    float verticalHalfFovRad = verticalFov / 2 * M_PI / 180.0f;

    auto it = std::lower_bound(config->azimuthBins.begin(), config->azimuthBins.end(), -horizontalHalfFovRad);
    int binIdx = std::distance(config->azimuthBins.begin(), --it);
    int azimuthMaxBin = config->numAzimuthBins - binIdx - 1;
    it = std::lower_bound(config->elevationBins.begin(), config->elevationBins.end(), -verticalHalfFovRad);
    binIdx = std::distance(config->elevationBins.begin(), --it);
    int elevationMaxBin = config->numElevationBins - binIdx - 1;
    int rangeMaxBin = static_cast<int>(std::ceil(range / config->rangeBinWidth));
    return clipHeatmapImage(image, azimuthMaxBin, elevationMaxBin, rangeMaxBin, config);
}

std::vector<float> coloradar::clipHeatmapImage(const std::vector<float>& image, const int& azimuthMaxBin, const int& elevationMaxBin, const int& rangeMaxBin, coloradar::RadarConfig* config) {
    int azimuthBinLimit = config->numAzimuthBins / 2;
    int elevationBinLimit = config->numElevationBins / 2;
    int rangeBinLimit = config->numRangeBins;
    if (! 0 <= azimuthMaxBin < azimuthBinLimit) {
        throw std::out_of_range("Invalid azimuthMaxBin selected: allowed selection from 0 to " + std::to_string(azimuthBinLimit - 1) + ", got " + std::to_string(azimuthMaxBin));
    }
    if (! 0 <= elevationMaxBin < elevationBinLimit) {
        throw std::out_of_range("Invalid elevationMaxBin selected: allowed selection from 0 to " + std::to_string(elevationBinLimit - 1) + ", got " + std::to_string(elevationMaxBin));
    }
    if (! 0 <= rangeMaxBin < rangeBinLimit) {
        throw std::out_of_range("Invalid rangeMaxBin selected: allowed selection from 0 to " + std::to_string(rangeBinLimit - 1) + ", got " + std::to_string(rangeMaxBin));
    }
    int azimuthLeftBin = azimuthBinLimit - azimuthMaxBin - 1;
    int azimuthRightBin = azimuthBinLimit + azimuthMaxBin;
    int elevationLeftBin = elevationBinLimit - elevationMaxBin - 1;
    int elevationRightBin = elevationBinLimit + elevationMaxBin;
    std::vector<float> clipped;
    for (int e = elevationLeftBin; e <= elevationRightBin; ++e) {
        for (int a = azimuthLeftBin; a <= azimuthRightBin; ++a) {
            for (int r = 0; r <= rangeMaxBin; ++r) {
                for (int n = 0; n < 2; ++n) {
                    int index = (((e * config->numAzimuthBins + a) * config->numRangeBins + r) * 2) + n;
                    clipped.push_back(image[index]);
                }
            }
        }
    }
    return clipped;
}
