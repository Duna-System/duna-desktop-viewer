#version 330
attribute vec3 vertex;
attribute vec3 color;
attribute float sizeIdx;

uniform mat4 mvpMatrix;
uniform float pointsize;
varying highp vec3 color_;

void main(void) {
  color_ = color;
  gl_PointSize = pointsize * sizeIdx;
  gl_Position = mvpMatrix * vec4(vertex, 1.0);
}