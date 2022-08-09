#include "point_picker.h"

template <typename PointT>
int PointPicker<PointT>::pickPoint(int mouse_x, int mouse_y, int width, int height, const QMatrix4x4& MVP, const float min_dist)
{

    // NDC (center of GLViewer and inside interval [-1,1])
    float ndc_x = (static_cast<float>(mouse_x) / static_cast<float>(width) - 0.5f) * 2.0f;
    float ndc_y = (0.5f - static_cast<float>(mouse_y) / static_cast<float>(height)) * 2.0f;

    // project to world coordinates (making interval z [-1.1])
    QVector4D ray0 = QVector4D(ndc_x, ndc_y, -1.0f, 1.0f);
    QVector4D ray1 = QVector4D(ndc_x, ndc_y, 0.0f, 1.0f);

    // getting ray in 3D world
    
    QMatrix4x4 MVP_inverse = MVP.inverted();
    QVector4D ray_world0 = MVP_inverse * ray0;
    QVector4D ray_world1 = MVP_inverse * ray1;

    // I dont understood that :O
    ray_world0.setW(1.0 / ray_world0.w());
    ray_world1.setW(1.0 / ray_world1.w());
    ray_world0.setX(ray_world0.x() * ray_world0.w());
    ray_world1.setX(ray_world1.x() * ray_world1.w());
    ray_world0.setY(ray_world0.y() * ray_world0.w());
    ray_world1.setY(ray_world1.y() * ray_world1.w());
    ray_world0.setZ(ray_world0.z() * ray_world0.w());
    ray_world1.setZ(ray_world1.z() * ray_world1.w());
    
    QVector3D ray_origin_point = QVector3D(ray_world0.x(), ray_world0.y(), ray_world0.z());
    QVector3D ray_direction = QVector3D(ray_world1.x() - ray_world0.x(), ray_world1.y() - ray_world0.y(), ray_world1.z() - ray_world0.z());
    ray_direction.normalize();    
    
    // get point picked
    int closestIndex = -1;
    float closestDist = std::numeric_limits<float>::max();

    if (m_hasOctree)
    {
        
        // Convert to eigen types.
        Eigen::Vector3f ray_direction_eigen(ray_direction.x(), ray_direction.y(), ray_direction.z());
        Eigen::Vector3f ray_origin_eigen(ray_origin_point.x(), ray_origin_point.y(), ray_origin_point.z());
        std::vector<int> intercepted_points_indices;
        
        int numInterceptedVoxels = m_octreeSearch->getIntersectedVoxelIndices(ray_origin_eigen, ray_direction_eigen, intercepted_points_indices, 0);
        // qDebug() << "numInterceptedVoxels: " << numInterceptedVoxels;
        // Get closest of all intersected
        for (int i = 0; i < numInterceptedVoxels; ++i)
        {            
            const int &probe_index = intercepted_points_indices[i];
            QVector3D point = QVector3D(m_cloud->points[probe_index].x, m_cloud->points[probe_index].y, m_cloud->points[probe_index].z); // generic point                                                                 // direction should be unity
            float pointDist = point.distanceToLine(ray_origin_point, ray_direction);
            // qDebug() << "pointDist: " << pointDist;
            if (pointDist < closestDist)
            {
                closestDist = pointDist;
                closestIndex = probe_index;
            }
        }
    }
    else
    {
        // Pick the closest point to line
        for (int i = 0; i < m_cloud->size(); ++i) // This is definetly a loop to be performed on the GPU
        {                        
            QVector3D query_point = QVector3D(m_cloud->points[i].x, m_cloud->points[i].y, m_cloud->points[i].z); // generic point
                                                                             // direction should be unity
            float pointDist = query_point.distanceToLine(ray_origin_point, ray_direction);

            if (pointDist < closestDist)
            {
                closestDist = pointDist;
                closestIndex = i;
            }
        }
    }

    if (closestDist < min_dist)
    {
        return closestIndex;
    }
    else
        return -1;
}


// Instantiate
template class PointPicker<pcl::PointXYZRGB>;