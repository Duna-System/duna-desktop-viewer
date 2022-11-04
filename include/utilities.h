#ifndef UTILITIES_H
#define UTILITIES_H

#include <QStringList>
#include <QFileDialog>
#include <QImage>
#include <QImageReader>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <chrono>
#include "cloud_loader.h"

namespace utilities
{
    /* Point Cloud */
    void loadPointCloudDialog(QWidget *window, CloudLoader& loader);

    /* Image */
    void loadImageDialog(QWidget *window, QImage &cloud);
    void loadImageFile(const std::string &filename, QImage &image);
    void loadImage(QString &filename, QImage &image);

    class Stopwatch
    {
    public:
        Stopwatch() = default;
        ~Stopwatch() = default;

        inline void enable()
        {
            is_enabled = true;
        }

        inline void disable()
        {
            is_enabled = false;
        }

        inline void tick()
        {
            if (!is_enabled)
                return;

            m_tick = std::chrono::high_resolution_clock::now();
        }

        // TODO decouble from <iostream>
        inline void tock(const std::string &message)
        {
            if (!is_enabled)
            {
                fprintf(stderr, "Warning, stopwatch not enabled\n");
                return;
            }

            const auto delta_tick = std::chrono::high_resolution_clock::now() - m_tick;
            fprintf(stderr, "'%s' took: %f [s]\n", message.c_str(), std::chrono::duration<double>(delta_tick).count());
        }

    private:
        // Debug timing
        std::chrono::time_point<std::chrono::high_resolution_clock> m_tick;
        bool is_enabled = true;
    };

}

#endif