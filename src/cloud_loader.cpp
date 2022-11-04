#include "cloud_loader.h"

// Searches for Intensity fields and renames it for fromPCLPointCloud2;
void CloudLoader::processIntensityField()
{
    needsIntensityConversion = false;
    for (auto &it : m_cloud->fields)
    {
        if (it.name.find("intensity") != std::string::npos)
        {
            std::cout << "Converting intensity";
            it.name = "intensity";
            needsIntensityConversion = true;
            break;
        }
    }
}

bool CloudLoader::loadPointCloud(const std::string &filename)
{
    const std::string extension = filename.substr(filename.find_last_of(".") + 1);
    bool success = true;
    if (extension == "pcd")
    {
        m_cloud.reset(new pcl::PCLPointCloud2);
        if (pcl::io::loadPCDFile(filename, *m_cloud) != 0)
        {
            success = false;
        }
    }
    if (extension == "ply")
    {
        m_cloud.reset(new pcl::PCLPointCloud2);
        if (pcl::io::loadPLYFile(filename, *m_cloud) != 0)
        {
            success = false;
        }
    }
    if (!success)
        throw std::runtime_error("Error loading point cloud.");

    processIntensityField();
    return success;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudLoader::createRGBCloud() const
{
    if (!m_cloud)
        return 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (needsIntensityConversion)
    {
        std::cout << "Converting intensity fields to RGB...\n";
        pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(*m_cloud, *intensity_cloud);

        rgb_cloud_ptr->resize(intensity_cloud->size());

        for (int i = 0; i < intensity_cloud->size(); ++i)
        {
            rgb_cloud_ptr->points[i].x = intensity_cloud->points[i].x;
            rgb_cloud_ptr->points[i].y = intensity_cloud->points[i].y;
            rgb_cloud_ptr->points[i].z = intensity_cloud->points[i].z;
            float r, g, b;
            Int2RGB_rviz(intensity_cloud->points[i].intensity,
                         rgb_cloud_ptr->points[i].r,
                         rgb_cloud_ptr->points[i].g,
                         rgb_cloud_ptr->points[i].b);
        }
    }
    else
    {
        pcl::fromPCLPointCloud2(*m_cloud, *rgb_cloud_ptr);
    }

    return rgb_cloud_ptr;
}

// pcl::PointCloud<pcl::PointXYZI>::Ptr CloudLoader::createIntensityCloud() const
// {
//     if (!m_cloud)
//         return 0;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromPCLPointCloud2(*m_cloud, *intensity_cloud_ptr);
//     return intensity_cloud_ptr;
// }