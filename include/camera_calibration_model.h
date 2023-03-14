#ifndef CAMERA_CALIBRATION_MODEL_H
#define CAMERA_CALIBRATION_MODEL_H

#include <duna/model.h>
#include <duna/so3.h>

struct CameraCalibrationModel : public duna::BaseModel<double>
{
    CameraCalibrationModel(const std::vector<Eigen::Vector4d> &point_list, const std::vector<Eigen::Vector2i> &pixel_list)
        : point_vector(point_list), pixel_vector(pixel_list)
    {
        if (pixel_list.empty())
            throw std::runtime_error("Empty pixel list");

        if (point_list.empty())
            throw std::runtime_error("Empty point list");

        if (pixel_list.size() != point_list.size())
            throw std::runtime_error("Different point sizes");

        camera_laser_frame_conversion.setIdentity();

        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
        camera_laser_frame_conversion.block<3, 3>(0, 0) = rot;

        camera_model << 586.122314453125, 0, 638.8477694496105, 0, 0,
            722.3973388671875, 323.031267074588, 0,
            0, 0, 1, 0;
    }

    inline void setup(const double *x) override
    {
        so3::convert6DOFParameterToMatrix(x, transform);
    }

    inline bool f(const double *x, double *residual, unsigned int index) override
    {
        Eigen::Vector3d out_pixel;
        out_pixel = camera_model * transform * camera_laser_frame_conversion * point_vector[index];
        residual[0] = pixel_vector[index][0] - (out_pixel[0] / out_pixel[2]);
        residual[1] = pixel_vector[index][1] - (out_pixel[1] / out_pixel[2]);
        return true;
    }

    Eigen::Matrix4d getFrameConversionMatrix() const
    {
        return camera_laser_frame_conversion;
    }

private:
    // input data

    const std::vector<Eigen::Vector4d> &point_vector;
    const std::vector<Eigen::Vector2i> &pixel_vector;

    Eigen::Matrix<double, 3, 4> camera_model;
    Eigen::Matrix4d camera_laser_frame_conversion;

    // Parameter to matrix conversion
    Eigen::Matrix4d transform;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif