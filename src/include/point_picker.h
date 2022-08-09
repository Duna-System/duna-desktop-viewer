#ifndef POINTPICKER_H
#define POINTPICKER_H

#include <QVector>
#include <QMatrix4x4>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <utilities.h>

template <typename PointT>
class PointPicker
{
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudTConstPtr = typename PointCloudT::ConstPtr;

    using OctreeType = pcl::octree::OctreePointCloudSearch<PointT>;
    using OctreeTypePtr = typename OctreeType::Ptr;

public:
    PointPicker() = default;
    PointPicker(int octree_build_threshold) : m_octree_build_threshold(octree_build_threshold)
    {
    }

    inline void setOctreeBuildThreshold(unsigned int octree_build_threshold) { m_octree_build_threshold = octree_build_threshold; }
    inline void setOctreeResolution(const double resolution)
    {
        m_octree_resolution = resolution;
        // TODO avoid recreation of octree..
        setCloud(m_cloud);

        // Code commented below does work
        // if (m_octreeSearch){
        //     m_octreeSearch->setResolution(m_octree_resolution);
        //     // If cloud is present, refresh computation
        //     if(m_cloud){
        //         m_octreeSearch->deleteTree();
        //         m_octreeSearch->setInputCloud(m_cloud);
        //         m_octreeSearch->addPointsFromInputCloud();
        //     }
        // }
    }

    inline void setCloud(const PointCloudTConstPtr& cloud)
    {
        m_cloud = cloud;

        if (m_cloud->size() > m_octree_build_threshold)
        {
            utilities::Stopwatch timer;
            timer.tick();
            qDebug() << "cloud has points > " << m_octree_build_threshold << " . Building octree..\n";
            m_octreeSearch.reset(new OctreeType(m_octree_resolution));
            m_octreeSearch->setInputCloud(m_cloud);
            m_octreeSearch->addPointsFromInputCloud();
            timer.tock("Octree Build\n");
            m_hasOctree = true;
        }
        else 
            m_hasOctree = false;
    }

    int pickPoint(int mouse_x, int mouse_y, int width, int height, const QMatrix4x4 &MVP, const float min_dist = 0.05f);

private:
    unsigned int m_octree_build_threshold = UINT_MAX;
    double m_octree_resolution = 0.05f;
    bool m_hasOctree = false;
    PointCloudTConstPtr m_cloud;
    OctreeTypePtr m_octreeSearch;
};

#endif