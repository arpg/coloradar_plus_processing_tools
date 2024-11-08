#ifndef PCL_FUNCTIONS_HPP
#define PCL_FUNCTIONS_HPP


template <coloradar::Pcl4dPointType PointT, template <coloradar::PclCloudType> class CloudT>
void coloradar::octreeToPcl(const octomap::OcTree& tree, CloudT<PointT>& cloud) {
    cloud.clear();
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        PointT point;
        octomap::point3d coords = it.getCoordinate();
        point.x = coords.x();
        point.y = coords.y();
        point.z = coords.z();
        point.intensity = it->getLogOdds();
        cloud.push_back(point);
    }
}

template <coloradar::PclPointType PointT, template <coloradar::PclCloudType> class CloudT>
void coloradar::filterFov(CloudT<PointT>& cloud, const float& horizontalFov, const float& verticalFov, const float& range) {
    coloradar::internal::filterFov<PointT, CloudT<PointT>>(cloud, horizontalFov, verticalFov, range);
}


struct PointKey {
    float x;
    float y;
    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};
struct PointKeyHash {
    std::size_t operator()(const PointKey& key) const {
        std::hash<float> hash_fn;
        return hash_fn(key.x) ^ (hash_fn(key.y) << 1);
    }
};


template <coloradar::Pcl4dPointType PointT, template <typename> class CloudT>
void collapseElevation(CloudT<PointT>& cloud, const float& elevationMin, const float& elevationMax) {
    if (elevationMin >= elevationMax) {
        throw std::invalid_argument("Invalid elevation range: elevationMin must be less than elevationMax.");
    }
    std::unordered_map<PointKey, PointT, PointKeyHash> max_intensity_map;
    max_intensity_map.reserve(cloud.size());

    for (const auto& point : cloud) {
        if (point.z >= elevationMin && point.z <= elevationMax) {
            PointKey key{point.x, point.y};
            auto iter = max_intensity_map.find(key);
            if (iter == max_intensity_map.end() || point.intensity > iter->second.intensity) {
                max_intensity_map[key] = point;
            }
        }
    }
    cloud.clear();
    cloud.reserve(max_intensity_map.size());
    for (const auto& [key, point] : max_intensity_map) {
        cloud.push_back(point);
    }
}

template <coloradar::PclPointType PointT, template <typename> class CloudT>
void collapseElevation(CloudT<PointT>& cloud) {
    std::unordered_map<PointKey, PointT, PointKeyHash> unique_points_map;
    unique_points_map.reserve(cloud.size());
    for (const auto& point : cloud) {
        PointKey key{point.x, point.y};
        if (unique_points_map.find(key) == unique_points_map.end()) {
            unique_points_map[key] = point;
        }
    }
    cloud.clear();
    cloud.reserve(unique_points_map.size());
    for (const auto& [key, point] : unique_points_map) {
        cloud.push_back(point);
    }
}


#endif
