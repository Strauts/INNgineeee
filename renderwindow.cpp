#include "renderwindow.h"
#include "mainwindow.h"
#include "material.h"
#include "camera.h"
#include "coreengine.h"
#include "ecsmanager.h"
#include "inputsystem.h"
#include "mousepicker.h"

#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLDebugLogger>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QDebug>


RenderWindow::RenderWindow(const QSurfaceFormat &format, MainWindow *mainWindow)
    : mContext{nullptr}, mInitialized{false}, mMainWindow{mainWindow}
{
    setSurfaceType(QWindow::OpenGLSurface);
    setFormat(format);
    mContext = new QOpenGLContext(this);    //Automatically deleted
    mContext->setFormat(requestedFormat());
    if (!mContext->create())
    {
        delete mContext;
        mContext = nullptr;
    }
}

RenderWindow::~RenderWindow()
{
}

void RenderWindow::init()
{
    //Some general stuff
    if (!mContext->makeCurrent(this))
    {
        emit error(tr("makeCurrent() failed"));
        return;
    }

    if (!mInitialized)
        mInitialized = true;

    initializeOpenGLFunctions();
    startOpenGLDebugger();

    //General OpenGL stuff:
    glEnable(GL_DEPTH_TEST);    //Enables depth sorting - must use GL_DEPTH_BUFFER_BIT in glClear
    glEnable(GL_CULL_FACE);     //Draws only front side of models
    glClearColor(0.4f, 0.4f, 0.4f,1.0f);    //Color used in glClear GL_COLOR_BUFFER_BIT

    checkForGLerrors();

    emit ready();   //Tell the mainWindow that init is finished
}

//Called to set up renderWindow before each frame
void RenderWindow::preRender(GLfloat deltaTime)
{
    if (mMousePickerOn)
        mCoreEngine->mMousePicker->update();
    mContext->makeCurrent(this); //must be called every frame (every time mContext->swapBuffers is called)
    initializeOpenGLFunctions();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

//Called after each rendered frame
void RenderWindow::postRender()
{
    glBindVertexArray(0);
    if (isExposed())
        mContext->swapBuffers(this);
}

void RenderWindow::startOpenGLDebugger()
{
    QOpenGLContext *temp = this->context();
    if (temp)
    {
        QSurfaceFormat format = temp->format();
        qDebug() << "Can this system run QOpenGLDebugLogger? :" << format.testOption(QSurfaceFormat::DebugContext);

        if (temp->hasExtension(QByteArrayLiteral("GL_KHR_debug")))
        {
            qDebug() << "System can log OpenGL errors!";
            mOpenGLDebugLogger = new QOpenGLDebugLogger(this);  //Will be deleted automatically
            if (mOpenGLDebugLogger->initialize()) //Initializes in the current context
                qDebug() << "Started OpenGL debug logger!";
            //Try to silence some messages that is not critical
            mOpenGLDebugLogger->disableMessages(QOpenGLDebugMessage::AnySource, QOpenGLDebugMessage::AnyType, QOpenGLDebugMessage::NotificationSeverity);
        }
    }
}

void RenderWindow::setCoreEngine(CoreEngine *coreEngine)
{
    mCoreEngine = coreEngine;
}

GLfloat RenderWindow::getMouseX()
{
    return mouseXlast;
}

GLfloat RenderWindow::getMouseY()
{
    return mouseYlast;
}

void RenderWindow::setShowAxis(bool showAxis)
{
    mShowAxis = showAxis;
}

void RenderWindow::setWireframe(bool wireframe)
{
    mWireframe = wireframe;
    if (mWireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);    //turn on wireframe mode
        glDisable(GL_CULL_FACE);
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);    //turn off wireframe mode
        glEnable(GL_CULL_FACE);
    }
}

void RenderWindow::checkForGLerrors()
{
    if (mOpenGLDebugLogger)
    {
        const QList<QOpenGLDebugMessage> messages = mOpenGLDebugLogger->loggedMessages();
        for (const QOpenGLDebugMessage &message : messages)
            qDebug() << message;
    }
    else
    {
        GLenum err = GL_NO_ERROR;
        while((err = glGetError()) != GL_NO_ERROR)
        {
            qDebug() << "glGetError returns " << err;
        }
    }
}

//This function is called from Qt when window is exposed / shown
void RenderWindow::exposeEvent(QExposeEvent *)
{
    if (!mInitialized)
        init();

    GLint retinaScale = static_cast<GLint>(devicePixelRatio());
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
}

void RenderWindow::mouseMoveEvent(QMouseEvent *event)
{
    mouseXlast = event->pos().x();
    mouseYlast = event->pos().y();
    mCoreEngine->mECSManager->mInputSystem->mouseMovement(event);
}

void RenderWindow::wheelEvent(QWheelEvent* event)
{
}

void RenderWindow::keyPressEvent(QKeyEvent *event)
{
    mCoreEngine->mECSManager->mInputSystem->keyPress(event);
}

void RenderWindow::keyReleaseEvent(QKeyEvent *event)
{
    mCoreEngine->mECSManager->mInputSystem->keyRelease(event);
}

void RenderWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        mMouseControl = true;
    }
    if(event->button() == Qt::LeftButton)
    {
        mMousePickerOn = true;
    }

    mCoreEngine->mECSManager->mInputSystem->mousePress(event);
}

void RenderWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        setCursor(Qt::ArrowCursor);
        mMouseControl = false;
    }
    if(event->button() == Qt::LeftButton)
    {
        mMousePickerOn = false;
    }

    mCoreEngine->mECSManager->mInputSystem->mouseRelease(event);
}
