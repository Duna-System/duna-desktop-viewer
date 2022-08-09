 #version 330
 varying highp vec3 color_;

  void main() {
    gl_FragColor = vec4(color_, 0.5);
  }