#ifndef SHADER_H
#define SHADER_H

#include <QOpenGLFunctions_4_1_Core>


class Shader : protected QOpenGLFunctions_4_1_Core
{
    public:
        //Constructor generates the shader on the fly
        Shader(const std::string vertexName, const std::string fragmentName);
        ~Shader();
        //Uses the current shader
        void Use();

        GLuint getProgram() const;
        std::string getName() const;

    private:
        GLuint program;
        std::string mName;
};

#endif
