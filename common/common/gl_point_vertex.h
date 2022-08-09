#ifndef GL_POINT_VERTEX_H
#define GL_POINT_VERTEX_H

#include <qopengl.h>
#include "gl_types.h"
#include <QOpenGLFunctions>
#include <QVector>

/* This Class defines a data buffer useful for interfacing with VBOs.
Template type should be the vertex structure intended. Refer to "gl_types.h" for examples.
*/
template <typename VertexType = GLPointVertex>
class GLPointBuffer
{
public:
    GLPointBuffer() = default;
    virtual ~GLPointBuffer() = default;

    inline void addPoint(const VertexType &point){ vertex_data.push_back(point); }

    inline void clear() {vertex_data.clear(); }

    // Return size occupied in bytes, useful for allocation
    inline unsigned int getSizeBytes() const { return (sizeof(VertexType) * vertex_data.size()); }

    // Return number of elements
    inline unsigned int getSize() const { return vertex_data.size(); }

    inline const void* getConstDataPtr() { return reinterpret_cast<const void*>(vertex_data.constData());}

    inline unsigned int getVertexLayoutSizeBytes() const { return sizeof(VertexType); }

protected:
    QVector<VertexType> vertex_data;
    //void setupVertexAttribs(); // not to be called yet    
};

#endif