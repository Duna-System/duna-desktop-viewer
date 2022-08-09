#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QImage>
#include <Eigen/Core>
#include <pcl/surface/texture_mapping.h>

template <typename PointInT>
class CloudTexture
{
public:

    using ImageConstPtr = std::shared_ptr<const QImage>;
    using PointCloudT = pcl::PointCloud<PointInT>;
    using PointCloudTConstPtr = typename PointCloudT::ConstPtr;
    using IntrinsicsMatrix = Eigen::Matrix<double, 3, 4>;
    using ExtrinsicsMatrix = Eigen::Matrix4f;
    using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

    CloudTexture() = default;
    virtual ~CloudTexture() = default;

    // Set image resolution (width, height)
    inline void setResolution(float width, float height) {
        m_width = width;
        m_height = height;
    }

    inline void setPointCloud(const PointCloudTConstPtr &cloud)
    {
        input_cloud = cloud;
    }

    inline void setImage(const ImageConstPtr &image)
    {
        input_image = image;
    }

    /*
                               [fx 0 cx 0]
    CameraMatrix/Intrinsics  = [0 fy cy 0]
                               [0  0  1 0]
    */
    inline void setIntrisicsMatrix(const IntrinsicsMatrix &matrix)
    {
        intrinsics = matrix;
        std::cout << "loaded intrinsics matrix: \n" << intrinsics << std::endl;
    }

    /*
                  [rxx rxy rxz Tx]
    Extrinsics  = [ryx ryy ryz Ty]
                  [rzx rzy rzz Tz]
                  [0     0   0  1]
    */
    inline void setExtrinsicsMatrix(const ExtrinsicsMatrix &matrix)
    {
        extrinsics = matrix;
        std::cout << "loaded extrinsics matrix: \n" << extrinsics << std::endl;
    }

    void texturize(PointCloudRGB &output);

private:
    PointCloudTConstPtr input_cloud;
    ImageConstPtr input_image;
    IntrinsicsMatrix intrinsics;
    ExtrinsicsMatrix extrinsics;
    float m_width;
    float m_height;
};

/*
extrinsics matrix:   0.999767 -0.0147186  0.0158089 -0.0191437
 0.0158089   0.997336 -0.0712094   0.113209
-0.0147186  0.0714427   0.997336  0.0274852
         0          0          0          1
*/