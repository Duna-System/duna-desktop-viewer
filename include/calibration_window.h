#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QPixmap>
#include <QImageReader>
#include <QDragEnterEvent>
#include <QMimeData>
#include <QScrollBar>
#include <QTime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "utilities.h"
#include "image_rectification.h"

#include "cloud_texture.h"
#include "qgl_viewer.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class CalibrationWindow;
}
QT_END_NAMESPACE

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT
    using PointT = pcl::PointXYZI; // TODO generalize
public:
    CalibrationWindow(QWidget *parent = nullptr);
    virtual ~CalibrationWindow();
    void dragEnterEvent(QDragEnterEvent *e);
    void dropEvent(QDropEvent *e);
    void setPointCloudFile(const std::string &filename);
    void setImageFile(const std::string &filename);

private:
    Ui::CalibrationWindow *ui;
    ImageRectification rectifier;

    void showMessage(const QString &text, bool print_debug = false);

protected:
    pcl::PointCloud<PointT>::Ptr m_cloud;
    std::shared_ptr<QImage> m_image;               // input image
    std::shared_ptr<const QImage> m_texture_image; // texture map image

    //
    std::vector<Eigen::Vector4d> m_point_list;
    std::vector<Eigen::Vector2i> m_pixel_list;

    // Viewer for the resulting cloud
    std::unique_ptr<QWidget> gl_texture_viewer;
    QGLViewer *gl_cast = nullptr;
    void resizeEvent(QResizeEvent *event) override;
    void closeEvent(QCloseEvent *event);

    bool m_pointpick_mode = false;
    bool m_pixelpick_mode = false;

public Q_SLOTS:
    void selectCameraModelPressed();
    void ToggleRectification();
    void selectFilePressed();
    void selectFilePressedImg();
    void optimizePressed();
    void setPointPickChecked(bool);
    void setPixelPickChecked(bool);
};
#endif // CALIBRATIONWINDOW_H
