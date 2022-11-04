#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QTimer>
#include <QMap>
#include <QPair>

#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "common/gl_point_vertex.h"
#include "common/gl_line_vertex.h"
#include "common/camera.h"
#include "common/gl_text2d.h"

#include <point_picker.h>

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)
QT_FORWARD_DECLARE_CLASS(Camera)

struct GridConfig
{
  GridConfig();

  int minX, maxX; // draw the grid from minX to maxX
  int minY, maxY; // ...and from minY to maxY
  int step;

  GLUColor color; // grid color
};

struct AxesConfig
{
  AxesConfig();

  float length;
  float arrowSize;

  GLUColor colorX, colorY, colorZ;
};

class QGLViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT
public: 
  using PointT = pcl::PointXYZRGB;
  using GLText2DPtr = std::shared_ptr<GLText2D>;
  QGLViewer(QWidget *parent = nullptr);
  virtual ~QGLViewer() override;

  QSize gl_size;

  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;
  void setGLsize(QSize size);
  QSize getGLsize() const;
  Camera *camera() const { return m_camera; }
  // void setData(const GLData &data);
  void setGridConfig(const GridConfig &grid);
  void setAxesConfig(const AxesConfig &axes);
  void setCloud(const pcl::PointCloud<PointT>::ConstPtr &cloud_);

  inline std::vector<pcl::PointXYZ> getPickedPointList() const 
  {
    return m_picked_point_list;
  }
  void setModePicking(bool modePicking_);
  int pointPicking(int mouse_x, int mouse_y, int width, int height, QMatrix4x4 MVP, pcl::PointCloud<PointT>::Ptr cloud, float min_dist);
  void markPointPicked(int pointPickedIdx, float size = 6.0);
  void resetLastMarkPointPicked();

public slots:
  void toggleModePicking();

protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int width, int height) override;
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

  PointPicker<PointT> m_point_picker;

  pcl::PointCloud<PointT>::Ptr m_cloud;

  bool modePicking = false;
  std::vector<int> pointsPickedIdx;
  std::vector<GLUColor> colorPointsPicked;

private:
  void initializeGridAndAxes();
  void setupGL();
  void setupVertexAttribs();

  QWidget *m_parent;

  QMap<int, bool> inputEntries;
  QPair<QPoint, QPoint> mousePositions;

  QPoint m_lastPos;
  GLLineBuffer<GLPointVertexUColor> m_data_lines; // TODO Change to lineBuffer class later on
  GLPointBuffer<GLPointVertexUColor> m_data_points;


  std::vector<pcl::PointXYZ> m_picked_point_list;
  // draw as triangles
  // QOpenGLVertexArrayObject m_trisVao;
  // QOpenGLBuffer m_trisVbo;

  // draw as lines: grid and axes
  QOpenGLVertexArrayObject m_linesVao;
  QOpenGLBuffer m_linesVbo;

  // draw points
  QOpenGLVertexArrayObject m_pointsVao;
  QOpenGLBuffer m_pointsVbo;
  GLfloat m_pointSize;

  bool m_drawGrid;
  int m_gridVertexIdx;
  GridConfig m_gridConfig;

  bool m_drawAxes;
  int m_axesVertexIdx;
  AxesConfig m_axesConfig;

  QOpenGLShaderProgram *m_program;

  Camera *m_camera;
  std::vector<GLText2DPtr> glTexts;

  int m_mvpMatrixLoc;
  int m_pointSizeLoc;

  QTimer *refreshRate;
  QTimer *poolRate;

private slots:
  void inputTreatment();
};