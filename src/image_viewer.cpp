#include "image_viewer.h"
// #include <QColorSpace>
#include <iostream>
#include <QDebug>

ImageViewer::ImageViewer(QWidget *parent)
    : QWidget(parent), layout(new QVBoxLayout(this)), imageLabel(new QLabel), scrollArea(new QScrollArea)
{
    layout->setSpacing(0);
    layout->setMargin(0);
    m_pixel_pick_pixmap.reset(new QPixmap);

    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);

    layout->addWidget(scrollArea);
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(imageLabel);
    scrollArea->setVisible(false);
}

void ImageViewer::setImage(const ImageConstPtr &image)
{
    qDebug() << image->format();
    m_image = image;
    m_pixel_pick_pixmap->convertFromImage(*m_image);
    imageLabel->setPixmap(*m_pixel_pick_pixmap);
    scrollArea->setVisible(true);
    imageLabel->adjustSize();
}

void ImageViewer::mousePressEvent(QMouseEvent *event)
{
    if (!m_modePicking)
        return;

    // float x = event->x();
    // float y = event->y();
    // float s_x = scrollArea->widget()->x();
    // float s_y = scrollArea->widget()->y();
    // std::cout << "Image size: " << imageLabel->height() << " " << imageLabel->width() << std::endl;
    // std::cout << "Image mouse press: " << x << "," << y << "\n";
    // std::cout << "Scroll Area  " << s_x << "," << s_y << "\n";

    if (event->buttons() & Qt::LeftButton)
    {

        QPoint image_pixel = convertScrollFrameToImageFrame(*scrollArea, event->pos());

        if ((image_pixel.x() >= 0 && image_pixel.x() <= imageLabel->width()) &&
            (image_pixel.y() >= 0 && image_pixel.y() <= imageLabel->height()))
        {
            std::cout << "Picked pixel: " << image_pixel.x() << "," << image_pixel.y() << std::endl;
            // TODO call callback
            m_pixel_list.push_back(image_pixel);
        }
        else
        {
            std::cerr << "Invalid pixel! Pick again\n";
        }
    }
    update();
}

void ImageViewer::paintEvent(QPaintEvent *event)
{
    if (!imageLabel->pixmap())
        return;

    QPainter painter(m_pixel_pick_pixmap.get());
    painter.setBrush(Qt::red);

    for (auto it = m_pixel_list.begin(); it != m_pixel_list.end(); it++)
    {
        // std::cout << "Pixel paint\n";
        painter.drawEllipse(*it, 5, 5);
        // TODO check corners
        QPoint offset(10, -5);
        std::string index(std::to_string(1 + it - m_pixel_list.begin()));
        painter.setPen(Qt::red);
        painter.drawText(*it + offset, index.c_str());
    }
    imageLabel->setPixmap(*m_pixel_pick_pixmap);
}

QPoint ImageViewer::convertScrollFrameToImageFrame(const QScrollArea &scrollArea, const QPoint &mousePosition)
{
    QPoint convertedPixel(mousePosition.x() - scrollArea.widget()->x(),
                          mousePosition.y() - scrollArea.widget()->y());

    // TODO checks
    return convertedPixel;
}

void ImageViewer::keyPressEvent(QKeyEvent *event)
{

    switch (event->key())
    {
    case Qt::Key_Control: // log current camera data
        // DISABLED IN FAVOR OF UI BUTTON
        // setModePicking(!m_modePicking);
        // qDebug() << ">>> Picking mode is " << (m_modePicking ? "Enabled" : "Disabled") << " <<<";
        break;
    case Qt::Key_R:
        if (!m_modePicking || m_pixel_list.size() == 0)
            return;
        // TODO callback
        m_pixel_list.pop_back();
        

        // Recover image and paint remaining points
        m_pixel_pick_pixmap->convertFromImage(*m_image);
        update();
        qDebug() << ">>>  reseting Pixel Picking";
        break;
    }
    //   update();
}
void ImageViewer::setModePicking(bool modePicking_)
{
    m_modePicking = modePicking_;
    qDebug() << ">>> Picking mode is " << (m_modePicking ? "Enabled" : "Disabled") << " <<<";
}

void ImageViewer::toggleModePicking()
{
    setModePicking(!m_modePicking);
    
}
