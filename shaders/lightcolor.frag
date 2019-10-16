#version 330 core
out vec4 fragColor;

struct Material {
    vec3 color;
    vec3 specularColor;
    float specularPower;
};

struct Light {
    vec3 normal_cs;
    vec3 ambient;
    vec3 diffuse;
    float intensity;
};

in vec3 fragPos_cs;
in vec3 normal_cs;

uniform vec3 cameraPosition;
uniform Material material;
uniform Light light;

void main()
{
    vec3 l_cs = light.normal_cs;
    vec3 vtoc_cs = normalize(-fragPos_cs);
    vec3 reflected_cs = reflect(-l_cs, normal_cs);

    float cosAlpha = clamp(dot(vtoc_cs, reflected_cs), 0, 1);
    float cosTheta = clamp(dot(normal_cs, l_cs), 0, 1);

    fragColor = vec4(light.ambient + material.color * light.intensity * light.diffuse * cosTheta +
                    (material.specularColor * light.diffuse * material.specularPower * pow(cosAlpha, 5)), 1);
}

