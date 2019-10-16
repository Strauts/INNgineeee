#include "shader.h"
#include "constants.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <QDebug>


Shader::Shader(const std::string vertexName, const std::string fragmentName) : mName{vertexName}
{
    initializeOpenGLFunctions();

    //Retrieve the vertex/fragment source code from filePath
    std::string vertexCode;
    std::string fragmentCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;

    //Ensures ifstream objects can throw exceptions:
    vShaderFile.exceptions (std::ifstream::badbit);
    fShaderFile.exceptions (std::ifstream::badbit);
    try
    {
        std::string vertexWithPath = gsl::shaderFilePath + vertexName;
        std::string fragmentWithPath = gsl::shaderFilePath + fragmentName;

        //Open files
        vShaderFile.open(vertexWithPath.c_str());
        fShaderFile.open(fragmentWithPath.c_str());
        std::stringstream vShaderStream, fShaderStream;

        //Read file's buffer contents into streams
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();

        //Close file handlers
        vShaderFile.close();
        fShaderFile.close();

        //Convert stream into string
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();
    }
    catch (std::ifstream::failure e)
    {
        qDebug() << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << QString::fromStdString(vertexName);
    }

    const GLchar *vShaderCode = vertexCode.c_str();
    const GLchar *fShaderCode = fragmentCode.c_str();

    //2. Compile shaders
    GLuint vertex, fragment;
    GLint success;
    GLchar infoLog[512];

    //Vertex Shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, nullptr);
    glCompileShader(vertex);
    //Print compile errors if any
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertex, 512, nullptr, infoLog);
        qDebug() << "ERROR::SHADER::VERTEX::COMPILATION_FAILED " << QString::fromStdString(vertexName) << "\n" << infoLog;
    }

    //Fragment Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, nullptr);
    glCompileShader(fragment);
    //Print compile errors if any
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragment, 512, nullptr, infoLog);
        qDebug() << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED " << QString::fromStdString(fragmentName) << "\n"  << infoLog;
    }

    //Shader Program
    this->program = glCreateProgram();
    glAttachShader(this->program, vertex);
    glAttachShader(this->program, fragment);
    glLinkProgram(this->program);
    // Print linking errors if any
    glGetProgramiv(this->program, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(this->program, 512, nullptr, infoLog);
        qDebug() << "ERROR::SHADER::PROGRAM::LINKING_FAILED " << QString::fromStdString(vertexName) << "\n" << infoLog;
    }

    // Delete the shaders as they're linked into our program now and no longer necessery
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    qDebug() << "Shader " << QString::fromStdString(vertexName) << "compiled sucessfully!";
}

Shader::~Shader()
{
}

void Shader::Use()
{
    glUseProgram(this->program);
}

GLuint Shader::getProgram() const
{
    return program;
}

std::string Shader::getName() const
{
    return mName;
}
