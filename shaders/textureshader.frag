#version 330

in vec2 UV;
uniform sampler2D textureSampler1;
uniform sampler2D textureSampler2;
out vec3 fragColor;

void main() {

    vec3 hund = texture( textureSampler1, UV ).rgb;
    vec3 uvMal = texture( textureSampler2, UV ).rgb;

   fragColor = mix(hund, uvMal, 0.2);
}
