#ifndef COREENGINE_H
#define COREENGINE_H

#include "gltypes.h"
#include <QBasicTimer>
#include <QTime>
#include <QObject>

class RenderWindow;
class MainWindow;
class ECSManager;
class InputSystem;
class MousePicker;
class LineDebug;

class CoreEngine : public QObject
{
    Q_OBJECT

    public:
        explicit CoreEngine(RenderWindow *renderWindow, MainWindow *mainWindow, QObject *parent = nullptr);

        RenderWindow *mRenderWindow{nullptr};    //The game window
        MainWindow *mMainWindow{nullptr};        //Main program window
        ECSManager *mECSManager{nullptr};
        MousePicker *mMousePicker{nullptr};
        LineDebug* mLineDebugClass{nullptr};

        static GLfloat getDeltaTime();
    private:
        void makeInitialEntities();
        void calculateFramerate(GLint msSinceLastFrame);
        void startLoop();
        void timerEvent(QTimerEvent *);

        QBasicTimer mTimer;     //Timer that drives the gameloop
        QTime mTimeStart;       //Time variable that reads the actual FPS
        static GLfloat mDeltaTime;  //Used to show FPS in MainWindow status bar
};

#endif // COREENGINE_H
