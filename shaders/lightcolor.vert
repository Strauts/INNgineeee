#version 330 core
layout(location = 0) in vec4 posAttr;
layout(location = 1) in vec3 normalAttr;
layout(location = 2) in vec2 uv;    //not used

out vec3 fragPos_cs;
out vec3 normal_cs;

uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;

void main()
{
    // Normal in cameraSpace
    vec4 n_cs = vMatrix * mMatrix * vec4(normalAttr, 0);
    vec4 v_cs = vMatrix * mMatrix * posAttr;
    normal_cs = n_cs.xyz;
    fragPos_cs = v_cs.xyz;
    gl_Position = pMatrix * v_cs;
}
