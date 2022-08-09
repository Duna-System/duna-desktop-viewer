#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QMimeData>

#include <calibration_window.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "utilities.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    using PointT = pcl::PointXYZI;
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow();
    void dragEnterEvent(QDragEnterEvent *e);
    void dropEvent(QDropEvent *e);

    // Set pointcloud file
    void setPointCloudFile(const std::string &filename);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow *ui;
    pcl::PointCloud<PointT>::Ptr cloud;

    std::unique_ptr<CalibrationWindow> calibration_window;

public Q_SLOTS:
    void selectFilePressed();
    void calibrationPressed();
};

#endif