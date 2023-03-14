#include "calibration_window.h"
#include "ui_calibration_window.h"

#include <duna/cost_function_numerical.h>
#include <duna/levenberg_marquadt.h>
#include <camera_calibration_model.h>

#include <duna/logger.h>
#include <opencv2/opencv.hpp>

CalibrationWindow::CalibrationWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::CalibrationWindow)
{
    ui->setupUi(this);
    connect(ui->actionAbrir, SIGNAL(triggered()), this, SLOT(selectFilePressed()));
    connect(ui->actionAbrir_Imagem, SIGNAL(triggered()), this, SLOT(selectFilePressedImg()));
    connect(ui->actionSet_camera_model, SIGNAL(triggered()), this, SLOT(selectCameraModelPressed()));
    // Buttons
    connect(ui->pushOptimize, SIGNAL(clicked()), this, SLOT(optimizePressed()));
    connect(ui->RectifyButton, SIGNAL(clicked()), this, SLOT(ToggleRectification()));
    // Connect data structure of optimization to image and opengl components. Maybe there is a better way
    // ui->GLviewer->setCalibrationDataPtr(&opt_data);
    // ui->imageViewer->setCalibrationDataPtr(&opt_data);

    // Point & Pixel pick
    connect(ui->PixelPick, SIGNAL(clicked()), ui->imageViewer, SLOT(toggleModePicking()));
    connect(ui->PointPick, SIGNAL(clicked()), ui->GLviewer, SLOT(toggleModePicking()));
    setAcceptDrops(true);
}

CalibrationWindow::~CalibrationWindow()
{
    delete ui;
}

void CalibrationWindow::resizeEvent(QResizeEvent *event) {}

void CalibrationWindow::setPointCloudFile(const std::string &filename)
{
    if (m_loader.loadPointCloud(filename))
        ui->GLviewer->setCloud(m_loader.createRGBCloud());
}

void CalibrationWindow::setImageFile(const std::string &filename)
{
    m_image.reset(new QImage);
    utilities::loadImageFile(filename, *m_image);
    ui->imageViewer->setImage(m_image);
}

void CalibrationWindow::selectFilePressedImg()
{
    m_image.reset(new QImage);
    m_texture_image = m_image;
    utilities::loadImageDialog(this, *m_image);
    ui->imageViewer->setImage(m_image);
}

void CalibrationWindow::selectFilePressed()
{
    utilities::loadPointCloudDialog(this, m_loader);
    ui->GLviewer->setCloud(m_loader.createRGBCloud());
}

void CalibrationWindow::dragEnterEvent(QDragEnterEvent *e)
{
    if (e->mimeData()->hasUrls())
    {
        e->acceptProposedAction();
    }
}

void CalibrationWindow::dropEvent(QDropEvent *e)
{
    QString firstFileName = e->mimeData()->urls()[0].toLocalFile();
    if (firstFileName.endsWith(".jpg", Qt::CaseSensitive) || firstFileName.endsWith(".jpeg", Qt::CaseSensitive) || firstFileName.endsWith(".png", Qt::CaseSensitive))
    {
        qDebug() << "e->mimeData()->urls()[0]: " << e->mimeData()->urls()[0];
        m_image.reset(new QImage);
        m_texture_image = m_image;
        utilities::loadImage(firstFileName, *m_image);
        ui->imageViewer->setImage(m_image);
    }
    else if (firstFileName.endsWith(".pcd", Qt::CaseSensitive) || (firstFileName.endsWith(".ply", Qt::CaseSensitive)))
    {
        foreach (const QUrl &url, e->mimeData()->urls())
        {
            QString fileName = url.toLocalFile();
            qDebug() << "Dropped file:" << fileName;
            // TODO verify if is cloud or image
            std::cout << "loading...\n";
            m_loader.loadPointCloud(fileName.toStdString());
            m_cloud = m_loader.createRGBCloud();
            ui->GLviewer->setCloud(m_cloud);
        }
    }
    else
        qDebug() << "Not supported files!";
}

void CalibrationWindow::optimizePressed()
{

    using PointT_ = pcl::PointXYZ;
    // Prepare model

    if (!rectifier.isInitialized())
    {
        showMessage("Camera model not lodaded.", true);
        return;
    }

    // Recover point & pixel list from other components
    const auto &pixel_list = ui->imageViewer->getPickedPixelList();
    m_pixel_list.clear();
    for (const auto &it : pixel_list)
    {
        const Eigen::Vector2i pt(it.x(), it.y());
        m_pixel_list.push_back(pt);
        qDebug() << pt[0] << "," << pt[1];
    }

    const auto &point_pist = ui->GLviewer->getPickedPointList();
    m_point_list.clear();
    for (const auto &it : point_pist)
    {
        const Eigen::Vector4d pt(it.x, it.y, it.z, 1);
        m_point_list.push_back(pt);
    }

    auto CameraModel = rectifier.getProjectionMatrix();

    std::cout << "Using camera model: " << CameraModel << std::endl;

    Eigen::Matrix<double, 6, 1> x0;
    x0.setZero();
    std::shared_ptr<CameraCalibrationModel> camera_model;
    try
    {
        camera_model.reset(new CameraCalibrationModel(m_point_list, m_pixel_list));
        duna::LevenbergMarquadt<double, 6> optimizer;
        auto cost = new duna::CostFunctionNumerical<double, 6, 2>(camera_model, m_pixel_list.size());
        optimizer.addCost(cost);
        auto status = optimizer.minimize(x0.data());
        std::stringstream ss;
        ss << x0;
        showMessage("Optimization Result: \n" + QString::fromStdString(ss.str()), true);
        qDebug() << "Iterations: " << optimizer.getExecutedIterations() << "/" << optimizer.getMaximumIterations();
        qDebug() << "Opt. Code: " << status;
    }
    catch (const std::runtime_error &ex)
    {
        showMessage("Optimizator exception: " + QString::fromStdString(ex.what()), true);
        return;
    }

    if (!gl_texture_viewer)
    {
        gl_texture_viewer.reset(new QGLViewer);
        gl_cast = reinterpret_cast<QGLViewer *>(gl_texture_viewer.get());
    }
    else
    {
        gl_cast->makeCurrent(); // That's really important in context of update viewer
    }

    CloudTexture<PointT> texturizer;
    texturizer.setIntrisicsMatrix(CameraModel);

    texturizer.setImage(m_texture_image);
    texturizer.setPointCloud(m_cloud);
    texturizer.setResolution(m_image->width(), m_image->height());
    std::cout << m_image->width() << "," << m_image->height() << std::endl;

    // Vector to matrix
    Eigen::Matrix4f extrinsics_converted = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q_(0, x0[3], x0[4], x0[5]);
    q_.w() = std::sqrt(1 - q_.dot(q_));
    q_.normalize();
    extrinsics_converted.topLeftCorner(3, 3) = q_.toRotationMatrix();
    extrinsics_converted(0, 3) = x0[0];
    extrinsics_converted(1, 3) = x0[1];
    extrinsics_converted(2, 3) = x0[2];
    extrinsics_converted = extrinsics_converted * camera_model->getFrameConversionMatrix().template cast<float>();
    texturizer.setExtrinsicsMatrix(extrinsics_converted);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    try
    {
        texturizer.texturize(*output);
        gl_cast->setCloud(output);
        gl_cast->show();
        gl_cast->doneCurrent();
    }
    catch (const std::exception &ex)
    {
        qDebug() << "texturizer exception: " << ex.what();
    }
}

void CalibrationWindow::selectCameraModelPressed()
{
    qDebug() << "set camera model pressed ";

    QStringList calib_files = QFileDialog::getOpenFileNames(
        this, "Select one Calibration file to open", "./", "model (*.yaml)", 0, QFileDialog::DontUseNativeDialog);

    if (calib_files.size())
    {
        try
        {
            rectifier.loadYamlConfig(calib_files.at(0).toStdString());
            showMessage("Camera model loaded successfully");
        }
        catch (const std::exception &ex)
        {
            qDebug() << ex.what();
            showMessage(ex.what());
        }
    }
}

void CalibrationWindow::ToggleRectification()
{
    if (!rectifier.isInitialized())
    {
        showMessage("Camera model not lodaded.");
        ui->RectifyButton->setChecked(false);
        return;
    }

    if (!m_image)
    {
        showMessage("Image not lodaded.");
        ui->RectifyButton->setChecked(false);
        return;
    }
    if (!ui->RectifyButton->isChecked())
    {
        ui->imageViewer->setImage(m_image);
        m_texture_image = m_image;
        return;
    }

    qDebug() << "rectifying image...";

    try
    {
        qDebug() << m_image->format();
        cv::Mat cv_mat(m_image->height(), m_image->width(), CV_8UC4, (uchar *)m_image->bits(), m_image->bytesPerLine());
        cv::Mat cv_undistorted;

        rectifier.rectifyImage(cv_mat, cv_undistorted);

        // This is a reference. If not copied, we're in trouble.
        QImage rectified_image_conversion((uchar *)cv_undistorted.data, cv_undistorted.cols, cv_undistorted.rows, cv_undistorted.step, QImage::Format_RGB32);

        // Copy to shared pointer
        std::shared_ptr<QImage> rectified_image(new QImage(rectified_image_conversion.copy()));

        // Set new texture image;
        m_texture_image = rectified_image;
        ui->imageViewer->setImage(rectified_image);

        qDebug() << "Image Rectified";
    }
    catch (std::exception &ex)
    {
        qDebug() << ex.what();
    }
}

void CalibrationWindow::showMessage(const QString &text, bool print_debug)
{
    QString text_with_timestamp("[" + QTime::currentTime().toString() + "]: " + text);
    ui->consoleLog->appendPlainText(text_with_timestamp);
    ui->consoleLog->verticalScrollBar()->setValue(ui->consoleLog->verticalScrollBar()->maximum());

    if (print_debug)
        qDebug() << text;
}

void CalibrationWindow::setPointPickChecked(bool checked)
{
    ui->PointPick->setChecked(checked);
}

void CalibrationWindow::setPixelPickChecked(bool checked)
{
    ui->PixelPick->setChecked(checked);
}

void CalibrationWindow::closeEvent(QCloseEvent *)
{
    if (gl_cast)
        gl_cast->close();
}