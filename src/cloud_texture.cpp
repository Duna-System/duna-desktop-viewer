#include "cloud_texture.h"
#include "pcl/io/pcd_io.h"

template <typename PointInT>
void CloudTexture<PointInT>::texturize(PointCloudRGB &output)
{
    if (!input_image)
        throw std::runtime_error("Input image not set.");

    pcl::copyPointCloud(*input_cloud, output);

    pcl::transformPointCloud(output, output, extrinsics);

    pcl::TextureMapping<pcl::PointXYZRGB> texture_map;
    pcl::TextureMapping<pcl::PointXYZRGB>::Camera camera;

    camera.focal_length_w = intrinsics(0, 0); // fx
    camera.focal_length_h = intrinsics(1, 1); // fy
    camera.center_h = intrinsics(1, 2); // cy
    camera.center_w = intrinsics(0, 2); // cx
    camera.width = m_width;
    camera.height = m_height;

    for (int i = 0; i < output.size(); ++i)
    {
        pcl::PointXYZRGB &point = output.points[i];
        Eigen::Vector2f uv_coords;
        texture_map.getPointUVCoordinates(point, camera, uv_coords);
        float u_ = uv_coords[0] * camera.width;
        float v_ = (1 - uv_coords[1]) * camera.height;

        if ((u_ >= 0 && u_ < camera.width) &&
            (v_ >= 0 && v_ < camera.height))
        {

            QRgb color = input_image->pixel(u_, v_); // Row, col
            // // // convert tripled to r,g,b
            point.r = qRed(color);
            point.g = qGreen(color);
            point.b = qBlue(color);

        }
        else
        {
            // Out of frame
        }

        // TODO remove occlusions with octree ?
    }

    // Back to laser frame
    pcl::transformPointCloud(output, output, extrinsics.inverse());
}

// Instantiate
template class CloudTexture<pcl::PointXYZI>;
template class CloudTexture<pcl::PointXYZRGB>;