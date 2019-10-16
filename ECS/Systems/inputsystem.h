#ifndef INPUTSYSTEM_H
#define INPUTSYSTEM_H

#include "systembase.h"
#include "componentbase.h"
#include "gltypes.h"
#include <QObject>
#include <QWidget>

class QGamepad;
class QKeyEvent;
class QMouseEvent;

class InputSystem : public SystemBase, public QWidget
{
    public:
        InputSystem();
        ~InputSystem() override;
        void update() override;

        //Event is coming from RenderWindow
        void keyPress(QKeyEvent *event);
        void keyRelease(QKeyEvent *event);
        void mousePress(QMouseEvent *event);
        void mouseRelease(QMouseEvent *event);
        void mouseMovement(QMouseEvent *event);

    public slots:
        void updateInputs(GLdouble &reciever, const GLdouble &input);
        void updateInputs(bool &reciever, const bool &input);

    private:
        void connectGamepad();

        QGamepad *mGamepad{nullptr};
        bool *mActionInputs;
        GLdouble *mAnalogInputs;

        InputComponent inputBuffer;
};

#endif // INPUTSYSTEM_H
