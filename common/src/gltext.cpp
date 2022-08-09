#include "common/gltext.h"
#include <QDebug>
#include <QPainter>

GLText::GLText(QOpenGLContext *context) : mIndexBuffer(QOpenGLBuffer::IndexBuffer)
{
    this->extern_context = context;
    modelMatrix.setToIdentity();
    mTexture = Q_NULLPTR;
}

GLText::~GLText()
{
    if (mTexture != Q_NULLPTR)
    {
        mTexture->destroy();
        delete mTexture;
    }
}

void GLText::initializeGL()
{

    initShaderProgram();

    GLTextVertex vertices[] = {
        {QVector3D(-0.5f, 0.5f, 0), QVector2D(0.0f, 0.0f)},
        {QVector3D(-0.5f, -0.5f, 0), QVector2D(0.0f, 1.0f)},
        {QVector3D(0.5f, 0.5f, 0), QVector2D(1.0f, 0.0f)},
        {QVector3D(0.5f, -0.5f, 0), QVector2D(1.0f, 1.0f)}};

    mBuffer.create();
    mBuffer.bind();
    mBuffer.allocate(vertices, sizeof(GLTextVertex) * 4);
    mBuffer.release();
    mVertexCount = 4;
    int indices[] = {0, 1, 2, 2, 3, 1};
    mIndexBuffer.create();
    mIndexBuffer.bind();
    mIndexBuffer.allocate(indices, 6 * sizeof(int));
    mIndexBuffer.release();
}

void GLText::resizeGL(int w, int h)
{
    mProjMatrix.setToIdentity();
    mProjMatrix.perspective(70.0f, GLfloat(w) / h, 0.001f, 10000.0f);
}
void GLText::renderText(QString text)
{
    mTexture = genTexture(512, 512, text, 60, Qt::white);
}

void GLText::renderText(QVector3D pos, QString text)
{
    GLTextVertex vertices[] = {
        {QVector3D(pos[0] - 0.5f, pos[1] + 0.5f, pos[2] + 0), QVector2D(0.0f, 0.0f)},
        {QVector3D(pos[0] - 0.5f, pos[1] - 0.5f, pos[2] + 0), QVector2D(0.0f, 1.0f)},
        {QVector3D(pos[0] + 0.5f, pos[1] + 0.5f, pos[2] + 0), QVector2D(1.0f, 0.0f)},
        {QVector3D(pos[0] + 0.5f, pos[1] - 0.5f, pos[2] + 0), QVector2D(1.0f, 1.0f)}};

    mBuffer.bind();
    mBuffer.write(0, vertices, 4 * sizeof(GLTextVertex));
    mBuffer.release();
     mTexture = genTexture(512, 512, text, 60, Qt::white);
}

void GLText::setModelMatrix(QMatrix4x4 newMatrix)
{
    modelMatrix = newMatrix * modelMatrix;
    this->update();
}
QMatrix4x4 GLText::getModelMatrix() { return modelMatrix; }

float m_frame = 0;
void GLText::paintGL(QMatrix4x4 cameraMatrix)
{

    if (mTexture == Q_NULLPTR)
        return;

    mShaderProgram->bind();

    mShaderProgram->setUniformValue(mMatrixLoc, cameraMatrix * modelMatrix);
    mShaderProgram->setUniformValue(mProjLoc, mProjMatrix);

    mTexture->bind();
    mBuffer.bind();
    mIndexBuffer.bind();
    QOpenGLFunctions *f = extern_context->functions();
    f->glVertexAttribPointer(mPosAttr, 3, GL_FLOAT, GL_FALSE, sizeof(GLTextVertex), 0);
    f->glVertexAttribPointer(mTexAttr, 2, GL_FLOAT, GL_FALSE, sizeof(GLTextVertex), reinterpret_cast<void *>(sizeof(QVector3D)));
    f->glEnableVertexAttribArray(mPosAttr);
    f->glEnableVertexAttribArray(mTexAttr);

    f->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    f->glDisableVertexAttribArray(mTexAttr);
    f->glDisableVertexAttribArray(mPosAttr);

    mIndexBuffer.release();
    mBuffer.release();
    mTexture->release();

    mShaderProgram->release();
}

static const char *vertexShaderSource =
    "attribute highp vec4 posAttr;\n"
    "attribute highp vec2 texcoord;\n"
    "varying highp vec2 v_texcoord;\n"
    "uniform highp mat4 matrix;\n"
    "uniform highp mat4 projMatrix;\n"
    "void main() {\n"
    "   v_texcoord = texcoord;\n"
    "   gl_Position = projMatrix * matrix * posAttr;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying highp vec2 v_texcoord;\n"
    "uniform sampler2D texture;\n"
    "void main() {\n"
    "   highp vec4 texColor = texture2D(texture, v_texcoord);\n"
    "   highp float alpha = texColor.a;\n"
    "   if(alpha < 0.2)\n"
    "       discard;\n"
    "   highp vec3 color = vec3(1.0,1.0,1.0);\n"
    "   gl_FragColor = texColor * vec4(color, 1.0);\n"
    "}\n";

void GLText::initShaderProgram()
{
    mShaderProgram = new QOpenGLShaderProgram(this);
    mShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    mShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    mShaderProgram->link();

    //    mShaderProgram->bind();
    mPosAttr = mShaderProgram->attributeLocation("posAttr");
    mTexAttr = mShaderProgram->attributeLocation("texcoord");
    mMatrixLoc = mShaderProgram->uniformLocation("matrix");
    mProjLoc = mShaderProgram->uniformLocation("projMatrix");
    //    mShaderProgram->release();
}

// void GLText::initUi()
// {
//     mButton = new QPushButton("rotate", this);
// #ifdef __ANDROID__
//     mButton->setGeometry(0, 0, 300, 100);
// #endif
//     connect(mButton, &QPushButton::clicked, this, &GLText::buttonClicked);
// }

QOpenGLTexture *GLText::genTexture(int width, int height, const QString &text, int textPixelSize, const QColor &textColor)
{
    QOpenGLTexture *texture = new QOpenGLTexture(QOpenGLTexture::Target2D);

    QImage img(width, height, QImage::Format_ARGB32_Premultiplied);
    img.fill(QColor(0, 0, 0, 0));

    QPainter painter;

    QFont font;
    painter.begin(&img);
    font.setPixelSize(textPixelSize);
    painter.setFont(font);
    QPen pen;
    pen.setColor(textColor);
    painter.setPen(pen);
    QTextOption option(Qt::AlignLeft | Qt::AlignTop);
    option.setWrapMode(QTextOption::WordWrap);
    QRectF rect(0, 0, width, height);
    painter.drawText(rect, text, option);
    painter.end();

    //    int mipLevelMax = width <= height ? log(width)/log(2) : log(height)/log(2);

    texture->setData(img);
    //    texture->setMipLevelRange(0, mipLevelMax);    //off mipmap
    texture->setMinificationFilter(QOpenGLTexture::Linear);
    texture->setMagnificationFilter(QOpenGLTexture::Linear);
    texture->setWrapMode(QOpenGLTexture::Repeat);

    return texture;
}

// void GLText::buttonClicked()
// {
//     if(!rot) {
//         rot = true;
//         mButton->setText("停止旋转");
//     } else {
//         rot = false;
//         mButton->setText("旋转");
//     }
// }
