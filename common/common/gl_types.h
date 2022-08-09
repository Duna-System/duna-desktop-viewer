#ifndef GL_TYPES_H
#define GL_TYPES_H

#include <qopengl.h>
#include <QVector3D>
#include <QVector2D>

/* Vertex Primitives */

struct GLPoint
{
    GLPoint() = default;
    GLPoint(GLfloat x_, GLfloat y_, GLfloat z_) : x(x_), y(y_), z(z_){};
    GLPoint(const QVector3D& vector) : x(vector[0]), y(vector[1]), z(vector[2]) {};

    union
    {
        GLfloat data[3];
        struct
        {
            GLfloat x;
            GLfloat y;
            GLfloat z;
        };
    };
};

// Generic Color layout
struct GLColor
{
    GLColor() = default;
    GLColor(GLubyte r_, GLubyte g_, GLubyte b_) : r(r_), g(g_), b(b_){};
    GLColor(const QVector3D& vector) : r(vector[0]), g(vector[1]), b(vector[2]) {};

    union
    {
        GLfloat data[3];
        struct
        {
            GLfloat r;
            GLfloat g;
            GLfloat b;
        };
    };
};

// Generic Color layout with uint8
struct GLUColor
{
    GLUColor() = default;
    GLUColor(GLubyte r_, GLubyte g_, GLubyte b_) : r(r_), g(g_), b(b_){};

    union
    {
        GLubyte data[3];
        struct
        {
            GLubyte r;
            GLubyte g;
            GLubyte b;
        };
    };
};

/* Vertex Layout Structures */

struct GLPointVertex
{
    GLPoint point; // 3x float
    GLColor color; // 3x uint8
    GLfloat size; // 1x float
};

// NOTE: Size of struct is 20 due to PADDING (https://stackoverflow.com/questions/119123/why-isnt-sizeof-for-a-struct-equal-to-the-sum-of-sizeof-of-each-member)
// Unsigned int as RGB color.
struct GLPointVertexUColor
{
    GLPoint point; // 3x float
    GLUColor color; // 3x uint8
    GLfloat size; // 1x float
};

struct GLTextVertex {
    QVector3D positon;
    QVector2D tex;
};

#endif