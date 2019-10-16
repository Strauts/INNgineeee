#version 330
layout(location = 0) in vec4 posAttr;
layout(location = 1) in vec4 colAttr;
layout(location = 2) in vec2 uv;    //not used

out vec4 col;
uniform mat4 matrix;

void main() {
   col = colAttr;
   gl_Position = matrix * posAttr;
}
