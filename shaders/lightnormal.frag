#version 330 core
out vec4 fragColor;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

struct Light {
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 fragPos;
in vec3 normal;

in mat3 normRotMat;
in vec2 UV;

uniform vec3 cameraPosition;
uniform Material material;
uniform Light light;
uniform sampler2D normalSampler;
//uniform sampler2D specularSampler;            //Ordner med normal først

void main()
{
    vec3 normalSample = texture(normalSampler, UV).rgb;
    normalSample = normalSample - 0.5;
    normalSample = normalSample * 2;
    
    normalSample = normRotMat * normalSample;
    
    // ambient
    vec3 ambient = light.ambient * material.ambient;

    // diffuse
    vec3 normalChecked = normalize(normalSample);
    vec3 lightDirection = normalize(light.position - fragPos);
    float diffuseStrenght = max(dot(normalChecked, lightDirection), 0.0);
    vec3 diffuse = light.diffuse * material.diffuse * diffuseStrenght;

    // specular
    vec3 viewDir = normalize(cameraPosition - fragPos);
    float specularStrenght = 0.0;
    if (diffuseStrenght > 0.0)  //this is necessary to avoid a specular impact on the backside of the mesh
    {
        vec3 reflectDir = reflect(lightDirection, normalChecked);  //reflect does not care about the direction of the normal so gives shine on backside also!
        specularStrenght = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    }
    vec3 specular = light.specular * material.specular * specularStrenght;

    vec3 result = ambient + diffuse + specular;
    fragColor = vec4(result, 1.0);
}



