#ifndef GL_LINE_VERTEX_H
#define GL_LINE_VERTEX_H

#include <qopengl.h>
#include "gl_types.h"
#include <QOpenGLFunctions>
#include <QVector>

/* This Class defines a data buffer useful for interfacing with VBOs.
Template type should be the vertex structure intended. Refer to "gl_types.h" for examples.
-- Line objects --*/
template <typename VertexType = GLPointVertex>
class GLLineBuffer 
{
public:
    GLLineBuffer(){};
    virtual ~GLLineBuffer(){};

    // Single color line
    inline void addLine(const GLPoint &line_a, const GLPoint &line_b, GLUColor color)
    {
        VertexType vtx;
        vtx.point = line_a;
        vtx.color = color;

        vertex_data.push_back(vtx);
        vtx.point = line_b;
        // color remains the same
        vertex_data.push_back(vtx);
    }

    inline void clear() { vertex_data.clear(); }

    // Return size occupied in bytes, useful for allocation
    inline unsigned int getSizeBytes()
    {
        return (sizeof(VertexType) * vertex_data.size());
    }

    inline void resizeLineVertexCount(unsigned int size_) { vertex_data.resize( size_ * sizeof(VertexType));}

    // Return number of elements
    inline unsigned int getSize()
    {
        return vertex_data.size();
    }

    inline const GLfloat *getConstDataPtr() { return reinterpret_cast<const GLfloat *>(vertex_data.constData()); }

protected:
    QVector<VertexType> vertex_data;
    void setupVertexAttribs(); // not to be called yet
};

#endif