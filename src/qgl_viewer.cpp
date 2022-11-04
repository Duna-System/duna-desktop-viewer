#include "qgl_viewer.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>

#include <cmath>
#include <iostream>

#include <utilities.h>

#include <calibration_window.h>
#include <memory>

#ifndef SHADER_PATH
#define DSHADER_PATH "./" // Defaults to current dir
#endif

// grid config defaults
GridConfig::GridConfig()
    : minX(-20), maxX(20),
      minY(-20), maxY(20),
      step(1),
      color(200, 200, 200)
{
}

// axes config defaults
AxesConfig::AxesConfig()
    : length(1.0f),
      arrowSize(0.1f),
      colorX(255, 0, 0),
      colorY(0, 255, 0),
      colorZ(0, 0, 255)
{
}

QGLViewer::QGLViewer(QWidget *parent)
    : m_parent(parent),
      m_drawGrid(true),
      m_gridVertexIdx(-1),
      m_gridConfig(),
      m_drawAxes(true),
      m_axesVertexIdx(-1),
      m_axesConfig(),
      m_program(nullptr),
      m_camera(new Camera)
{
  QSurfaceFormat format;
  format.setDepthBufferSize(16);
  format.setSamples(8);
  setFormat(format);

  Camera *camera = this->camera();

  CameraConfig config;
  config.c_mode = CameraMode::Target;
  config.p_mode = ProjectionMode::Perspective;
  config.fov = 45;
  config.nearPlane = 0.01;
  config.farPlane = 200.0f;

  // initialTranslation in world coords
  config.initialTranslation = QVector3D(-5, 2, 1);

  // how to interpret world coordinates to directions - Z is up
  config.WorldForward = QVector3D(1, 0, 0);
  config.WorldRight = QVector3D(0, -1, 0);
  config.WorldUp = QVector3D(0, 0, 1);

  camera->setTarget(1.0, 0, 0);

  camera->setConfig(config);

  refreshRate = new QTimer(this);
  poolRate = new QTimer(this);
  connect(poolRate, SIGNAL(timeout()), this, SLOT(inputTreatment()));
  connect(poolRate, SIGNAL(timeout()), this, SLOT(update()));
  poolRate->start(1000 / 120);
  refreshRate->start(1000 / 60);
  setMouseTracking(false);

  m_cloud.reset(new pcl::PointCloud<PointT>);
}

QGLViewer::~QGLViewer()
{
  if (m_program == nullptr)
    return;

  makeCurrent();
  // m_trisVbo.destroy();
  m_linesVbo.destroy();
  m_pointsVbo.destroy();
  delete m_program;
  m_program = nullptr;
  doneCurrent();

  delete refreshRate;
  delete poolRate;
  delete m_camera;
  m_camera = nullptr;
}

QSize QGLViewer::minimumSizeHint() const
{
  return QSize(50, 50);
}

QSize QGLViewer::sizeHint() const
{
  return QSize(800, 600);
}

void QGLViewer::setGLsize(QSize size) { gl_size = size; }
QSize QGLViewer::getGLsize() const { return gl_size; }


void QGLViewer::setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_)
{
  m_cloud->resize(cloud_->size());
  pcl::copyPointCloud(*cloud_, *m_cloud);
  m_point_picker.setOctreeBuildThreshold(1000000);
  m_point_picker.setCloud(m_cloud);

  setupGL();
}

void QGLViewer::setGridConfig(const GridConfig &grid)
{
  m_gridConfig = grid;
  setupGL();
  // update();
}

void QGLViewer::setAxesConfig(const AxesConfig &axes)
{
  m_axesConfig = axes;
  setupGL();
  // update();
}

void QGLViewer::initializeGL()
{
  initializeOpenGLFunctions();

  // glClearColor(0.9, 0.9, 0.9, 1);
  glClearColor(0.0, 0.0, 0.0, 0);
  glDepthFunc(GL_LESS);

  m_program = new QOpenGLShaderProgram;
  m_program->addShaderFromSourceFile(QOpenGLShader::Vertex, QString(SHADER_PATH) + "./source.vert");
  m_program->addShaderFromSourceFile(QOpenGLShader::Fragment, QString(SHADER_PATH) + "./source.frag");

  m_program->bindAttributeLocation("vertex", 0);
  m_program->bindAttributeLocation("color", 1);
  m_program->bindAttributeLocation("sizeIdx", 2);

  if (!m_program->link())
    std::cerr << "ERROR: failed to link: " << m_program->log().toStdString();

  m_mvpMatrixLoc = m_program->uniformLocation("mvpMatrix");
  m_pointSizeLoc = m_program->uniformLocation("pointsize");
  m_pointSize = 1.0;

  // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
  // implementations this is optional and support may not be present
  // at all. Nonetheless the below code works in all cases and makes
  // sure there is a VAO when one is needed.
  // if (!m_trisVao.create() || !m_linesVao.create() || !m_pointsVao.create())

  if (!m_linesVao.create() || !m_pointsVao.create())
    std::cerr << "ERROR: faild to create vertex array object" << std::endl;

  // if (!m_trisVbo.create() || !m_linesVbo.create() || !m_pointsVbo.create())
  if (!m_linesVbo.create() || !m_pointsVbo.create())
    std::cerr << "ERROR: failed to create vertex buffer object" << std::endl;

  setupGL();

  m_camera->reset();
}

void QGLViewer::initializeGridAndAxes()
{
  if (m_gridVertexIdx > -1)
  {
    // if there were already a grid and axes, delete them before rebuilding
    m_data_lines.resizeLineVertexCount(m_gridVertexIdx);
  }

  m_gridVertexIdx = m_data_lines.getSize();

  // setup grid
  for (int x = m_gridConfig.minX; x <= m_gridConfig.maxX; x += m_gridConfig.step)
  {
    for (int y = m_gridConfig.minY; y <= m_gridConfig.maxY; y += m_gridConfig.step)
    {
      // parallel to x

      GLUColor color;
      color.r = m_gridConfig.color.r;
      color.g = m_gridConfig.color.g;
      color.b = m_gridConfig.color.b;
      m_data_lines.addLine(GLPoint(-x, y, 0), GLPoint(x, y, 0), color);

      // parallel to y
      m_data_lines.addLine(GLPoint(x, -y, 0), GLPoint(x, y, 0), color);
    }
  }

  // m_axesVertexIdx = m_data_lines.lineVertexCount();
  m_axesVertexIdx = m_data_lines.getSize();

  // setup coordinate axes
  const float &length = m_axesConfig.length;
  const float &arrSize = m_axesConfig.arrowSize;

  // x (red)
  m_data_lines.addLine(GLPoint(-length, 0, 0), GLPoint(length, 0, 0), m_axesConfig.colorX);

  // arrow
  m_data_lines.addLine(GLPoint(length, 0, 0), GLPoint(length - arrSize, arrSize / 2, 0), m_axesConfig.colorX);
  m_data_lines.addLine(GLPoint(length, 0, 0), GLPoint(length - arrSize, -arrSize / 2, 0), m_axesConfig.colorX);

  // y (green)
  m_data_lines.addLine(GLPoint(0, -length, 0), GLPoint(0, length, 0), m_axesConfig.colorY);

  // arrow
  m_data_lines.addLine(GLPoint(0, length, 0), GLPoint(arrSize / 2, length - arrSize, 0), m_axesConfig.colorY);
  m_data_lines.addLine(GLPoint(0, length, 0), GLPoint(-arrSize / 2, length - arrSize, 0), m_axesConfig.colorY);

  // z (blue)
  m_data_lines.addLine(GLPoint(0, 0, -length), GLPoint(0, 0, length), m_axesConfig.colorZ);

  // arrow
  m_data_lines.addLine(GLPoint(0, 0, length), GLPoint(arrSize / 2, 0, length - arrSize), m_axesConfig.colorZ);
  m_data_lines.addLine(GLPoint(0, 0, length), GLPoint(-arrSize / 2, 0, length - arrSize), m_axesConfig.colorZ);
}

void QGLViewer::setupGL()
{
  if (m_program == nullptr)
    return;

  // std::cout << "setupGL\n";
  initializeGridAndAxes();

  // m_trisVao.bind();
  m_program->bind();

  // Setup our vertex buffer objects.

  // Lines
  m_linesVao.bind();
  m_linesVbo.bind();
  m_linesVbo.allocate(m_data_lines.getConstDataPtr(), m_data_lines.getSizeBytes());
  setupVertexAttribs();
  m_linesVbo.release();
  m_linesVao.release();

  // PointCloud
  if (m_cloud) // check if allocated
  {
    // std::cout << "adding pts\n";
    m_pointsVao.bind();
    m_pointsVbo.bind();
    // m_data_points.setupVertexAttribs(); // wont work..
    setupVertexAttribs();
    m_data_points.clear();

    for (int i = 0; i < m_cloud->size(); ++i)
    {
      GLPointVertexUColor vertex;
      vertex.point.x = m_cloud->points[i].x;
      vertex.point.y = m_cloud->points[i].y;
      vertex.point.z = m_cloud->points[i].z;
      // 0 < vertex color < 255
      vertex.color.r = m_cloud->points[i].r;
      vertex.color.g = m_cloud->points[i].g;
      vertex.color.b = m_cloud->points[i].b;

      vertex.size = 1.0;
      m_data_points.addPoint(vertex);
    }
    // std::cout << "m_data_points ready\n";
    m_pointsVbo.allocate(m_data_points.getConstDataPtr(), m_data_points.getSizeBytes());
    m_pointsVbo.release();
    m_pointsVao.release();
  }

  m_program->release();
  // QVector3D text_pos (1,1,1);
  // glTexts[0].renderText(text_pos, QString::number(999));

  std::cout << "done\n";
}

// TODO maybe we can automate this method by teplating the vertex ?
void QGLViewer::setupVertexAttribs()
{
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);

  GLsizei stride = m_data_points.getVertexLayoutSizeBytes();
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);                             // Position
  glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, stride, reinterpret_cast<void *>(12)); // Color
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void *>(16));        // Size
}

void QGLViewer::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  // glEnable(GL_MULTISAMPLE);
  // glEnable(GL_CULL_FACE);
  glEnable(GL_PROGRAM_POINT_SIZE); // Need to set point size in shader

  // Render all text
  for (int i = 0; i < glTexts.size(); ++i)
  {
    // qDebug() << "renreding text: " << i << "\n";
    glTexts[i]->paintGL(m_camera->toMatrix());
  }

  m_program->bind();
  m_program->setUniformValue(m_mvpMatrixLoc, m_camera->toMatrix());
  m_program->setUniformValue(m_pointSizeLoc, m_pointSize);

  /* It doesn't matter if the vertex attributes are all from one buffer or multiple buffers,
   * and we don't need to bind any particular vertex buffer when drawing; all the glDraw* functions
   * care about is which vertex attribute arrays are enabled.
   */

  // m_trisVao.bind();
  // // render as wireframe
  // // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  // glDrawArrays(GL_TRIANGLES, 0, m_data.triangleVertexCount());
  // m_trisVao.release();

  glLineWidth(2);
  m_linesVao.bind();
  glDrawArrays(GL_LINES, 0, m_gridVertexIdx);
  m_linesVao.release();

  if (m_drawGrid && m_gridVertexIdx > -1)
  {
    glLineWidth(0.5f);
    m_linesVao.bind();
    glDrawArrays(GL_LINES, m_gridVertexIdx, m_axesVertexIdx - m_gridVertexIdx);
    m_linesVao.release();
  }

  if (m_drawAxes && m_axesVertexIdx > -1)
  {
    glLineWidth(5);
    m_linesVao.bind();
    glDrawArrays(GL_LINES, m_axesVertexIdx, m_data_lines.getSize() - m_axesVertexIdx);
    m_linesVao.release();
  }

  if (1)
  {
    // std::cout << "drawing pts: " << m_data.pointsVertexCount() << std::endl;
    m_pointsVao.bind();
    m_pointsVbo.bind(); // TODO no idea why vao has not stored states already
    setupVertexAttribs();
    glDrawArrays(GL_POINTS, 0, m_data_points.getSize());
    m_pointsVbo.release();
    m_pointsVao.release();
  }

  m_program->release();
}

void QGLViewer::resizeGL(int w, int h)
{
  m_camera->setAspectRatio(GLfloat(w) / h);
  setGLsize(QSize(w, h));
}

void QGLViewer::keyReleaseEvent(QKeyEvent *event)
{
  inputEntries[event->key()] = false;
}

void QGLViewer::keyPressEvent(QKeyEvent *event)
{
  inputEntries[event->key()] = true;

  // Simple hotkeys goes in normal events
  switch (event->key())
  {
  case Qt::Key_Minus:
    if (m_pointSize > 1.0)
      m_pointSize -= 1.0;
    break;
  case Qt::Key_Plus:
    if (m_pointSize < 20.0)
      m_pointSize += 1.0;
    break;
  case Qt::Key_I:
    m_drawAxes = !m_drawAxes;
    break;
  case Qt::Key_U:
    m_drawGrid = !m_drawGrid;
    break;

  case Qt::Key_0:
    m_camera->reset();
    break;
  case Qt::Key_P:
    m_camera->setProjectionMode(ProjectionMode::Perspective);
    break;
  case Qt::Key_O:
    m_camera->setProjectionMode(ProjectionMode::Orthographic);
    break;
  case Qt::Key_J:
    m_camera->setCameraMode(CameraMode::Free);
    setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);
    break;
  case Qt::Key_K:
    m_camera->setCameraMode(CameraMode::Target);
    setMouseTracking(false);
    this->setCursor(Qt::ArrowCursor);

    break;
  case Qt::Key_L: // log current camera data
    qDebug() << *m_camera;
    break;
  case Qt::Key_Control: // log current camera data
                        // DISABLED IN FAVOR OF UI UTTON
    // setModePicking(!modePicking);
    // qDebug() << ">>> Picking mode is " << (modePicking ? "Enabled" : "Disabled") << " <<<";
    break;
  case Qt::Key_R: // log current camera data
    if (!modePicking)
      return;
    qDebug() << ">>>  reseting Points Picking";

    makeCurrent(); // this avoid lost the context when cleaning all glTexts
    resetLastMarkPointPicked();
    doneCurrent();
    break;
  }

  // update();
}

void QGLViewer::mousePressEvent(QMouseEvent *event)
{

  m_lastPos = event->pos();

  if (modePicking && event->buttons() & Qt::LeftButton)
  {

    if (!m_point_picker.hasCloud())
    {
      qDebug() << "Not point cloud is set!\n";
      return;
    }

    // int pointPickedIdx = pointPicking(event->x(), event->y(), getGLsize().width(), getGLsize().height(), m_camera->toMatrix(), m_cloud, 0.1f); // Cannot be too low
    utilities::Stopwatch timer;
    timer.tick();
    int pointPickedIdx = m_point_picker.pickPoint(event->x(), event->y(), getGLsize().width(), getGLsize().height(), m_camera->toMatrix());
    timer.tock("Point Picking");

    if (pointPickedIdx > 0)
    {
      // Extremely important..
      makeCurrent();
      markPointPicked(pointPickedIdx);
      doneCurrent();
    }
    else
    {
      qDebug() << "No point picked";
    }
  }
  // update();
}

void QGLViewer::mouseReleaseEvent(QMouseEvent *event)
{
  inputEntries[event->button()] = false;
  this->setCursor(Qt::ArrowCursor);
}

void QGLViewer::inputTreatment()
{

  if (m_camera->cameraMode() != CameraMode::Free)
    return;

  mousePositions.first = this->rect().center();
  mousePositions.second = mapFromGlobal(QCursor::pos());
  QVector2D mouseMovement(mousePositions.first - mousePositions.second);
  QCursor::setPos(mapToGlobal(this->rect().center()));

  float &&dx = mouseMovement.x();
  float &&dy = mouseMovement.y();

  m_camera->rotate(0.1 * dx, m_camera->worldUpVector());
  m_camera->rotate(0.1 * dy, m_camera->rightVector());

  float speedMultiplier = 1;

  if (inputEntries[Qt::Key_Shift])
  {
    speedMultiplier = 3;
  }
  else if (inputEntries[Qt::Key_Control])
  {
    speedMultiplier = 0.1;
  }

  // First Person Movement
  if (inputEntries[Qt::Key_W])
  {
    m_camera->translate(m_camera->forwardVector() * 0.05 * speedMultiplier);
  }
  if (inputEntries[Qt::Key_A])
  {
    m_camera->translate(m_camera->rightVector() * -0.05 * speedMultiplier);
  }
  if (inputEntries[Qt::Key_S])
  {
    m_camera->translate(m_camera->forwardVector() * -0.05 * speedMultiplier);
  }
  if (inputEntries[Qt::Key_D])
  {
    m_camera->translate(m_camera->rightVector() * 0.05 * speedMultiplier);
  }
  if (inputEntries[Qt::Key_Q])
  {
    m_camera->translate(m_camera->upVector() * 0.05 * speedMultiplier);
  }
  if (inputEntries[Qt::Key_E])
  {
    m_camera->translate(m_camera->upVector() * -0.05 * speedMultiplier);
  }

  // update();
}

void QGLViewer::mouseMoveEvent(QMouseEvent *event)
{
  if (m_camera->cameraMode() != CameraMode::Target)
    return;

  float dx = event->x() - m_lastPos.x();
  float dy = event->y() - m_lastPos.y();

  QCursor::setPos(mapToGlobal(m_lastPos));

  if (event->modifiers() & Qt::ShiftModifier)
  {
    dx /= 4;
    dy /= 4;
  }

  int upDown = m_camera->upsideDown() ? -1 : 1;

  if (event->buttons() & Qt::LeftButton)
  {
    this->setCursor(Qt::BlankCursor);
    if (m_camera->cameraMode() == CameraMode::Free)
    {
      m_camera->rotate(-0.2f * dx, m_camera->worldUpVector());
      m_camera->rotate(-0.2f * dy, m_camera->worldRightVector());
    }
    else
    {
      // if the up vector actually points down, reverse rotation
      m_camera->rotate(-0.2f * dx, upDown * m_camera->worldUpVector());
      m_camera->rotate(-0.2f * dy, upDown * QVector3D::crossProduct(m_camera->forwardVector(), m_camera->worldUpVector()));
    }
  }
  else if (event->buttons() & Qt::MiddleButton)
  {
    if (m_camera->cameraMode() == CameraMode::Free)
    {
      m_camera->rotate(0.2f * dx, m_camera->forwardVector());
      m_camera->rotate(-0.2f * dy, m_camera->rightVector());
    }
    else
    {
      m_camera->rotate(-0.2f * dx, m_camera->forwardVector());
      m_camera->rotate(-0.2f * dy, upDown * QVector3D::crossProduct(m_camera->forwardVector(), m_camera->worldUpVector()));
    }
  }
  else if (event->buttons() & Qt::RightButton)
  {
    if (m_camera->cameraMode() == CameraMode::Free)
    {
      dx *= -1;
      dy *= -1;
    }
    float t_factor = 20.0;
    m_camera->translate(-dx / t_factor * m_camera->rightVector());
    m_camera->translate(dy / t_factor * m_camera->upVector());
  }

  // update();
}

void QGLViewer::wheelEvent(QWheelEvent *event)
{
  if (m_camera->cameraMode() != CameraMode::Target)
    return;
  QPoint numDeg = event->angleDelta();
  float dist = (m_camera->translation() - m_camera->target()).length();
  // std::cout << "dist: " << dist << "\n";
  if (numDeg.isNull())
    return;

  float factor = 0.1 * sqrtf(dist);

  if (event->modifiers() & Qt::ShiftModifier)
    factor /= 10;

  if (numDeg.y() < 0)
    factor = -factor;

  m_camera->translate(factor * m_camera->forwardVector());

  event->accept();
  // update();
}

void QGLViewer::markPointPicked(int pointPickedIdx, float pointSize)
{
  // change point picked in VBO
  m_pointsVbo.bind();
  GLPointVertexUColor pt_picked;
  m_pointsVbo.read(sizeof(GLPointVertexUColor) * pointPickedIdx, &pt_picked, sizeof(pt_picked));

  // avoiding repeat points
  if (pt_picked.color.r == 255 && pt_picked.color.g == 255 && pt_picked.color.b == 255)
    return;

  colorPointsPicked.push_back(pt_picked.color);
  pt_picked.color.r = 255;
  pt_picked.color.g = 255;
  pt_picked.color.b = 255;
  pt_picked.size = pointSize == 0 ? 1 : pointSize;                                                // change 7th index of buffer (size)
  m_pointsVbo.write(sizeof(GLPointVertexUColor) * pointPickedIdx, &pt_picked, sizeof(pt_picked)); // changing point at index pointPickedIdx in VBO
  m_pointsVbo.release();

  // create text instance
  QVector3D text_pos(m_cloud->points[pointPickedIdx].x, m_cloud->points[pointPickedIdx].y, m_cloud->points[pointPickedIdx].z);
  GLText2DPtr pointText = std::make_shared<GLText2D>(this->context());
  pointText->renderText(text_pos, QString::number(glTexts.size() + 1));
  glTexts.push_back(pointText);

  // saving point picked
  const pcl::PointXYZ pickedPoint(m_cloud->points[pointPickedIdx].x, m_cloud->points[pointPickedIdx].y, m_cloud->points[pointPickedIdx].z);
  m_picked_point_list.push_back(pickedPoint);
  pointsPickedIdx.push_back(pointPickedIdx);
  std::cout << pickedPoint << std::endl;
}

void QGLViewer::resetLastMarkPointPicked()
{
  // remove last text instance
  if (glTexts.size() == 0)
    return;
  glTexts.pop_back();

  // return size of point
  m_pointsVbo.bind();
  GLPointVertexUColor pt_picked;
  int lastPointPickedIdx = pointsPickedIdx.back();
  m_pointsVbo.read(sizeof(GLPointVertexUColor) * lastPointPickedIdx, &pt_picked, sizeof(pt_picked));
  GLUColor lastPointColor = colorPointsPicked.back();
  // std::cout << lastPointColor;
  pt_picked.color.r = lastPointColor.r;
  pt_picked.color.g = lastPointColor.g;
  pt_picked.color.b = lastPointColor.b;
  pt_picked.size = 1.0;                                                                               // change 7th index of buffer (size)
  m_pointsVbo.write(sizeof(GLPointVertexUColor) * lastPointPickedIdx, &pt_picked, sizeof(pt_picked)); // changing point at index lastPointPickedIdx in VBO
  m_pointsVbo.release();

  // remove last point picked
  m_picked_point_list.pop_back();
  pointsPickedIdx.pop_back();
}

void QGLViewer::setModePicking(bool modePicking_)
{
  modePicking = modePicking_;
  qDebug() << ">>> Point Picking mode is " << (modePicking ? "Enabled" : "Disabled") << " <<<";

  // TODO figure out a way to reflect the modePicking state on checkbox. Code below does not work. Also should only work when parent is calibration window
  // CalibrationWindow *parent = reinterpret_cast<CalibrationWindow *>(m_parent);

  // parent->togglePointPickChecked(modePicking);
}

void QGLViewer::toggleModePicking()
{
  setModePicking(!modePicking);
}