#ifndef IMAGE_RECTIFICATION_H
#define IMAGE_RECTIFICATION_H

#include <iostream>
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

/* ImageRectification provied useful methos for loading calibration files / camera models for rectifing images */
class ImageRectification 
{   
    using EigenMatrix34d = Eigen::Matrix<double,3,4>;
    public:
    ImageRectification() : isModelLoaded(false) {}
    virtual ~ImageRectification () {}

    bool isInitialized() const
    {
        return isModelLoaded;
    }

    
    void loadYamlConfig(const std::string& config_file);
    void rectifyImage(const cv::Mat& raw, cv::Mat& rectified) const;
    EigenMatrix34d getProjectionMatrix() const;

    private:
    cv::Mat_<double> camera_D;
    cv::Matx33d camera_R;
    cv::Matx33d camera_K;
    cv::Matx34d camera_P;

    cv::Mat mapx,mapy;
    bool isModelLoaded;

    template <class T>
    void fromStdVector(const std::vector<double>& vector_, T& converted_) const;
    


};


#endif