#ifndef GLTEXT2D_H
#define GLTEXT2D_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QMatrix4x4>
#include <QVector>
#include <QPushButton>

#include <memory>

#include "gl_types.h"

// This class should not inherit from QOpenGLWidget, because it is not a widget at all. 
// It could have its own shaders, memory and functionalities, but it shoud provide the 'render' method as an interface to its owners with openGL calls.
class GLText2D : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    GLText2D(const QOpenGLContext *context);
    GLText2D(const GLText2D &) = default;
    virtual ~GLText2D();

    // QOpenGLWidget interface
    // protected:

    void resizeGL(int w, int h) override;
    void paintGL(QMatrix4x4 cameraMatrix);
    // void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;
    void setModelMatrix(QMatrix4x4 newMatrix);
    QMatrix4x4 getModelMatrix();
    void renderText(QString text);
    void renderText(QVector3D pos, QString text);

private:
    void initializeGL() override;
    void initShaderProgram();
    // void initUi();
    void genTexture(int width, int height, const QString &text, int textPixelSize, const QColor &textColor);

    // private slots:
    //     // void buttonClicked();

private:
    QOpenGLShaderProgram *mShaderProgram;
    QMatrix4x4 mProjMatrix;

    GLuint mPosAttr;
    GLuint mTexAttr;
    GLuint mMatrixLoc;
    GLuint mProjLoc;
    GLuint mTextPos;

    QOpenGLBuffer mBuffer;
    QOpenGLBuffer mIndexBuffer;
    GLuint mVertexCount;

    std::unique_ptr<QOpenGLTexture> mTexture;
    const QOpenGLContext * const extern_context;
    QMatrix4x4 modelMatrix;

    // ui
    QPushButton *mButton;
    bool rot;
};

#endif // GLTEXT_H
