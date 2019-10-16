#ifndef RENDERWINDOW_H
#define RENDERWINDOW_H

#include "constants.h"
#include <QWindow>
#include <QOpenGLFunctions_4_1_Core>

class QOpenGLContext;
class MainWindow;
class QOpenGLDebugLogger;
class Camera;
class CoreEngine;

class RenderWindow : public QWindow, protected QOpenGLFunctions_4_1_Core
{
    Q_OBJECT
    public:
        RenderWindow(const QSurfaceFormat &format, MainWindow *mainWindow);
        ~RenderWindow() override;
        QOpenGLContext* context() {return mContext;}
        void exposeEvent(QExposeEvent *) override;

        void setWireframe(bool wireframe);
        void setShowAxis(bool showAxis);
        void setCoreEngine(CoreEngine *coreEngine);

        GLfloat getMouseX();
        GLfloat getMouseY();

        void checkForGLerrors();

        void preRender(GLfloat deltaTime);
        void postRender();
        bool mMousePickerOn{false};
    signals:
        void ready();
        void error(const QString &msg);

    private:
        void init();
        void startOpenGLDebugger();

        QOpenGLContext *mContext{nullptr};
        CoreEngine *mCoreEngine{nullptr};
        bool mInitialized{false};
        MainWindow *mMainWindow{nullptr};
        QOpenGLDebugLogger *mOpenGLDebugLogger{nullptr};

        bool mWireframe{false};
        bool mShowAxis{true};
        GLint mouseXlast{0};
        GLint mouseYlast{0};
        bool mMouseControl{false};

        GLint renderLoop{0};

        //For camera control
        GLfloat mHorizontalAngle{3.14f};
        GLfloat mVerticalAngle{0.f};
        GLfloat mDistance{40.f};

        bool mForward{false};
        bool mBackward{false};
        bool mRight{false};
        bool mLeft{false};

    protected:
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void keyPressEvent(QKeyEvent *event) override;
        void keyReleaseEvent(QKeyEvent *event) override;
        void wheelEvent(QWheelEvent *event) override;
};

#endif // RENDERWINDOW_H
