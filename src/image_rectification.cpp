#include "image_rectification.h"

/* Convert vector to cv type */
template <class T>
void ImageRectification::fromStdVector(const std::vector<double> &vector_, T &converted_) const
{
    T data_converted(vector_.data());
    converted_ = data_converted;
}

template <>
void ImageRectification::fromStdVector<cv::Mat_<double>>(const std::vector<double> &vector_, cv::Mat_<double> &converted_) const
{
    cv::Mat_<double> data_converted(vector_);
    converted_ = data_converted;
}

void ImageRectification::loadYamlConfig(const std::string &config_file)
{
    YAML::Node config = YAML::LoadFile(config_file);

    double image_width = config["image_width"].as<double>();
    double image_height = config["image_height"].as<double>();

    std::vector<double> camera_matrix_data = config["camera_matrix"]["data"].as<std::vector<double>>();
    fromStdVector(camera_matrix_data, camera_K);

    std::vector<double> distortion_coeficients_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
    fromStdVector(distortion_coeficients_data, camera_D);

    std::vector<double> rectification_matrix_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
    fromStdVector(rectification_matrix_data, camera_R);

    std::vector<double> projection_matrix_data = config["projection_matrix"]["data"].as<std::vector<double>>();
    fromStdVector(projection_matrix_data, camera_P);

    // std::cout << camera_K << std::endl;
    // std::cout << camera_D << std::endl;
    // std::cout << camera_R << std::endl;

    cv::Size size(image_width, image_height);

    cv::initUndistortRectifyMap(camera_K, camera_D, camera_R, camera_P, size, CV_16SC2, mapx, mapy);

    // std::cout << camera_P << std::endl;

    isModelLoaded = true;
}

void ImageRectification::rectifyImage(const cv::Mat &raw, cv::Mat &rectified) const
{
    if (!isModelLoaded)
        throw std::runtime_error("Camera model not loaded");

    std::cout << "UNDISTORTING...\n";
    // std::cout << camera_K << std::endl;
    // std::cout << camera_D << std::endl;
    // std::cout << camera_P << std::endl;

    if (rectified.depth() == CV_32F || rectified.depth() == CV_64F)
        cv::remap(raw, rectified, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, std::numeric_limits<float>::quiet_NaN());
    else
        cv::remap(raw, rectified, mapx, mapy, cv::INTER_LINEAR);

    // std::cout << camera_K << std::endl;
    // std::cout << camera_D << std::endl;
    // std::cout << camera_P << std::endl;
}

ImageRectification::EigenMatrix34d ImageRectification::getProjectionMatrix() const
{
    EigenMatrix34d matrix;
    cv2eigen(camera_P, matrix);
    return matrix;
}
