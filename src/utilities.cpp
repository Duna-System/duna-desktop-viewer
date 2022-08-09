#include "utilities.h"
#include <pcl/cloud_iterator.h>
namespace utilities
{
    // This is where we read and convert pointcloud files
    template <typename PointT>
    int loadPointCloudFile(const std::string &filename, pcl::PointCloud<PointT> &cloud)
    {
        if (filename.substr(filename.find_last_of(".") + 1) == "pcd")
            return pcl::io::loadPCDFile(filename, cloud);
        else if (filename.substr(filename.find_last_of(".") + 1) == "ply")
        {
            pcl::PCLPointCloud2::Ptr cloud_(new pcl::PCLPointCloud2);
            int retval = pcl::io::loadPLYFile(filename, *cloud_);
            if (retval)
                return retval;
            // Remap intensity field
            for (auto &it : cloud_->fields)
            {
                if (it.name.find("intensity") != std::string::npos)
                {
                    it.name = "intensity";
                    break;
                }
            }
            pcl::fromPCLPointCloud2(*cloud_, cloud);
            return retval;
        }
        // Error
        return 1;
    }

    // TODO avoid duplicate code
    void loadImageFile(const QString &filename, QImage &image)
    {
        QImageReader reader(filename);
        reader.setAutoTransform(true);
        image = reader.read();
    }

    void loadImageFile(const std::string &filename, QImage &image)
    {
        QImageReader reader(QString::fromStdString(filename));
        reader.setAutoTransform(true);
        image = reader.read();
    }

    template <typename PointT>
    void loadPointCloudDialog(QWidget *window, pcl::PointCloud<PointT> &cloud)
    {
        std::cout << "file pressed\n";
        QStringList pointcloud_files = QFileDialog::getOpenFileNames(
            window, "Select one file to open", "./", "PointClouds (*.pcd *.ply)", 0, QFileDialog::DontUseNativeDialog);

        // print all
        for (int i = 0; i < pointcloud_files.size(); ++i)
        {
            std::cout << pointcloud_files.at(i).toStdString() << std::endl;
        }

        // select only 1st
        std::cout << "loading...\n";
        if (pointcloud_files.size())
        {
            int retval = loadPointCloudFile(pointcloud_files.at(0).toStdString(), cloud);
            if (retval)
                std::cerr << "Point cloud load failure\n";
            else
                std::cout << "Loaded " << cloud.size() << " points\n";
        }
    }

    void loadImageDialog(QWidget *window, QImage &image)
    {
        std::cout << "file pressed img\n";
        QStringList img_files = QFileDialog::getOpenFileNames(
            window, "Select one IMG file to open", "./", "Image (*.jpg *.png)", 0, QFileDialog::DontUseNativeDialog);

        for (int i = 0; i < img_files.size(); ++i)
        {
            std::cout << img_files.at(i).toStdString() << std::endl;
        }

        if (img_files.size())
        {
            std::cout << "loading...\n";
            QImageReader reader(img_files.at(0));
            // reader.setAutoTransform(true);
            // image = reader.read();
            loadImageFile(img_files.at(0), image);
            std::cout << "loaded image.\n";
        }
    }

    void loadImage(QString &filename, QImage &image)
    {
        QImageReader reader(filename);
        reader.setAutoTransform(true);
        image = reader.read();
    }

    // Instantiate
    template void loadPointCloudDialog<pcl::PointXYZI>(QWidget *window, pcl::PointCloud<pcl::PointXYZI> &cloud);
}
