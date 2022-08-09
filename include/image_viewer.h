#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QImage>
#include <QLabel>
#include <QWidget>
#include <QScrollArea>
#include <QImageReader>
#include <QImage>
#include <QAction>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QPainter>

#include <memory>


class ImageViewer : public QWidget
{
    Q_OBJECT
    using ImagePtr = std::shared_ptr<QImage>;
    using ImageConstPtr = std::shared_ptr<const QImage>;

    using PixMapPtr = std::shared_ptr<QPixmap>;

public:
    ImageViewer(QWidget *parent = nullptr);
    void loadFile(const QString &);
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void setModePicking(bool modePicking_);
    void setImage(const ImageConstPtr &image);

    // Return list of currently picked pixels
    inline std::vector<QPoint> getPickedPixelList() const 
    {
        return m_pixel_list;
    }

    ImageConstPtr getImage() const { return m_image; }

public slots:
    void toggleModePicking();

private:
    QPoint convertScrollFrameToImageFrame(const QScrollArea &scrollArea, const QPoint &mousePosition);
    std::vector<QPoint> m_pixel_list;
    QVBoxLayout *layout;
    ImageConstPtr m_image;
    QLabel *imageLabel;
    QScrollArea *scrollArea;
    PixMapPtr m_pixel_pick_pixmap;
    bool m_modePicking = false;
};

#endif
