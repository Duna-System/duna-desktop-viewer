#include "common/gl_text2d.h"
#include <QDebug>
#include <QPainter>

GLText2D::GLText2D(const QOpenGLContext *context) : mIndexBuffer(QOpenGLBuffer::IndexBuffer),
                                                    extern_context(context)
{
    modelMatrix.setToIdentity();
    initializeGL();
}

GLText2D::~GLText2D()
{
    if (!mTexture)
    {
        mTexture->destroy();
    }
}

void GLText2D::initializeGL()
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

void GLText2D::resizeGL(int w, int h)
{
    mProjMatrix.setToIdentity();
    mProjMatrix.perspective(70.0f, GLfloat(w) / h, 0.001f, 10000.0f);
}

void GLText2D::renderText(QVector3D pos, QString text)
{

    mShaderProgram->bind();
    QVector4D text4(pos[0], pos[1], pos[2], 1);
    mShaderProgram->setUniformValue(mTextPos, text4);
    genTexture(512, 512, text, 60, Qt::white);
}

void GLText2D::renderText(QString text)
{
    genTexture(512, 512, text, 60, Qt::white);
}

void GLText2D::setModelMatrix(QMatrix4x4 newMatrix)
{
    // modelMatrix = newMatrix * modelMatrix;  // TODO why multiply for current model matrix ?
    modelMatrix = newMatrix;
    // this->update(); // Need to call this ?
}
QMatrix4x4 GLText2D::getModelMatrix() { return modelMatrix; }

void GLText2D::paintGL(QMatrix4x4 cameraMatrix)
{

    if (!mTexture)
        return;

    mShaderProgram->bind();

    mShaderProgram->setUniformValue(mMatrixLoc, cameraMatrix);

    // mShaderProgram->setUniformValue(mProjLoc, mProjMatrix);

    mTexture->bind();
    mBuffer.bind();
    mIndexBuffer.bind();
    QOpenGLFunctions *f = extern_context->functions();
    f->glEnableVertexAttribArray(mPosAttr);
    f->glEnableVertexAttribArray(mTexAttr);

    f->glVertexAttribPointer(mPosAttr, 3, GL_FLOAT, GL_FALSE, sizeof(GLTextVertex), 0);
    f->glVertexAttribPointer(mTexAttr, 2, GL_FLOAT, GL_FALSE, sizeof(GLTextVertex), reinterpret_cast<void *>(sizeof(QVector3D)));

    f->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    f->glDisableVertexAttribArray(mTexAttr);
    f->glDisableVertexAttribArray(mPosAttr);

    mIndexBuffer.release();
    mBuffer.release();
    mTexture->release();

    mShaderProgram->release();
}

static const char *vertexShaderSource =
    "#version 330 core\n"
    "attribute highp vec4 posAttr;\n"
    "attribute highp vec2 texcoord;\n"
    "varying highp vec2 v_texcoord;\n"
    "uniform highp mat4 matrix;\n"
    "uniform highp vec4 textPos;\n"
    "highp vec4 tf_Pos;\n"
    // "uniform highp mat4 projMatrix;\n"
    "void main() {\n"
    "   v_texcoord = texcoord;\n"
    "   tf_Pos = matrix * textPos;\n"
    "   if (gl_VertexID == 0)\n"
    "   gl_Position = tf_Pos;\n"
    "   if (gl_VertexID == 1)\n"
    "   gl_Position = vec4(tf_Pos.x,tf_Pos.y-3, 1, tf_Pos.w);\n"
    "   if (gl_VertexID == 2)\n"
    "   gl_Position = vec4(tf_Pos.x+3,tf_Pos.y, 1, tf_Pos.w);\n"
    "   if (gl_VertexID == 3)\n"
    "   gl_Position = vec4(tf_Pos.x+3,tf_Pos.y-3, 1, tf_Pos.w);\n"
    "}\n";

static const char *fragmentShaderSource =
    "#version 330 core\n"
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

void GLText2D::initShaderProgram()
{
    mShaderProgram = new QOpenGLShaderProgram(this);
    mShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    mShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    mShaderProgram->link();

    //    mShaderProgram->bind();
    mPosAttr = mShaderProgram->attributeLocation("posAttr");
    mTexAttr = mShaderProgram->attributeLocation("texcoord");
    mMatrixLoc = mShaderProgram->uniformLocation("matrix");
    mTextPos = mShaderProgram->uniformLocation("textPos");
    // mProjLoc = mShaderProgram->uniformLocation("projMatrix");
    //    mShaderProgram->release();
}

// void GLText2D::initUi()
// {
//     mButton = new QPushButton("rotate", this);
// #ifdef __ANDROID__
//     mButton->setGeometry(0, 0, 300, 100);
// #endif
//     connect(mButton, &QPushButton::clicked, this, &GLText2D::buttonClicked);
// }

void GLText2D::genTexture(int width, int height, const QString &text, int textPixelSize, const QColor &textColor)
{
    mTexture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));

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

    mTexture->setData(img);
    //    texture->setMipLevelRange(0, mipLevelMax);    //off mipmap
    mTexture->setMinificationFilter(QOpenGLTexture::Linear);
    mTexture->setMagnificationFilter(QOpenGLTexture::Linear);
    mTexture->setWrapMode(QOpenGLTexture::Repeat);
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
