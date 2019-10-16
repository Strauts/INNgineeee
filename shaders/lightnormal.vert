#version 330 core
layout(location = 0) in vec4 posAttr;
layout(location = 1) in vec3 normalAttr;
layout(location = 2) in vec2 uv;    //not used


out vec3 fragPos;
out vec3 normal;

out vec2 UV;
out mat3 normRotMat;

uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;

void main()
{
    fragPos = vec3(mMatrix * posAttr).xyz;  //fragment position in world space
    normal = mat3(transpose(inverse(mMatrix))) * normalAttr;    //should be done on CPU
    
    vec3 biTangent = vec3(0, -1, 0);

    vec3 tangent = vec3(1, 0, 0);
        
    
    //calculate tangents used to rotate normalMap to fit the vertex normals.
    if(normal == biTangent){                //exeptions for when object points up/down, using fixed values
        biTangent = vec3(0, 0, -1);
        
    } else if(normal == -biTangent){
        biTangent = vec3(0, 0, 1);
        
    } else {
                                            //biTangent will point as close to down as possible
        biTangent = normalize(biTangent - normal * dot(biTangent, normal));
                                            //tangent decied by cross between normal and biTangent
        tangent = cross(normal, biTangent);
    }
    normRotMat = mat3(tangent.x, tangent.y, tangent.z, biTangent.x, biTangent.y, biTangent.z, normal.x, normal.y, normal.z);
    
    //gl_Position = pMatrix * vMatrix * mMatrix * posAttr;;
    gl_Position = pMatrix * vMatrix * vec4(fragPos, 1.0);   //not sure if this is quicker...
    
    UV = uv;
}
