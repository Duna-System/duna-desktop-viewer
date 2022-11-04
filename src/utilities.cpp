#include "utilities.h"

namespace utilities
{

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

    void loadPointCloudDialog(QWidget *window, CloudLoader& loader)
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
            loader.loadPointCloud(pointcloud_files.at(0).toStdString());
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
}
