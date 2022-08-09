#ifndef GLTEXT_H
#define GLTEXT_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QMatrix4x4>
#include <QVector>
#include <QPushButton>

#include "gl_types.h"

class GLText :  public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    GLText(QOpenGLContext* context);
    ~GLText();

    // QOpenGLWidget interface
// protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL(QMatrix4x4 cameraMatrix);
    // void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;
    void setModelMatrix(QMatrix4x4 newMatrix);
    QMatrix4x4 getModelMatrix();
    void renderText(QString text);
    void renderText(QVector3D pos, QString text);
    
private:
    void initShaderProgram();
    // void initUi();
    QOpenGLTexture* genTexture(int width, int height, const QString& text, int textPixelSize, const QColor &textColor);

// private slots:
//     // void buttonClicked();

private:
    QOpenGLShaderProgram *mShaderProgram;
    QMatrix4x4 mProjMatrix;

    GLuint mPosAttr;
    GLuint mTexAttr;
    GLuint mMatrixLoc;
    GLuint mProjLoc;

    QOpenGLBuffer mBuffer;
    QOpenGLBuffer mIndexBuffer;
    GLuint mVertexCount;

    QOpenGLTexture *mTexture;
    QOpenGLContext* extern_context;
    QMatrix4x4 modelMatrix;

    //ui
    QPushButton *mButton;
    bool rot;
};



#endif // GLTEXT_H
