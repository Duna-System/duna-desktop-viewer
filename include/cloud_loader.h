#ifndef CLOUDLOADER
#define CLOUDLOADER

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include "colormap.hpp"

class CloudLoader 
{
    public:
    CloudLoader() = default;
    virtual ~CloudLoader() = default;

    // Loads supported point cloud types.
    bool loadPointCloud(const std::string& filename);

    // Point Cloud Factories
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createRGBCloud() const;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr createIntensityCloud() const ;

    private:
    pcl::PCLPointCloud2::Ptr m_cloud;
    void processIntensityField();
    bool needsIntensityConversion;
};

#endif