#include "inputsystem.h"
#include "componentbase.h"
#include "ecsmanager.h"
#include <QGamepad>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QCoreApplication>
#include <QDebug>


InputSystem::InputSystem() : SystemBase()
{
    mGamepad = new QGamepad;
    connectGamepad();
    qDebug() << "Done setting up Inputsystem!";
}

InputSystem::~InputSystem()
{
    delete mGamepad;
}

//Send input to all entities that have an input component attached...
void InputSystem::update()
{
    ECSManager *manager = ECSManager::getInstance();

    for (InputComponent &inputComp : manager->mInputComponents)
    {
        inputBuffer.ownerEntityID = inputComp.ownerEntityID;
        inputComp = inputBuffer;
    }
}

void InputSystem::keyPress(QKeyEvent *event)
{
    if (event->key() == Qt::Key_W)
        inputBuffer.mForward = -1.0;
    if (event->key() == Qt::Key_S)
        inputBuffer.mForward = 1.0;
    if (event->key() == Qt::Key_D)
        inputBuffer.mRight = 1.0;
    if (event->key() == Qt::Key_A)
        inputBuffer.mRight = -1.0;
    if (event->key() == Qt::Key_Space)
        inputBuffer.mSpace = true;
    if (event->key() == Qt::CTRL)
        inputBuffer.mCtrl = true;
    if (event->key() == Qt::SHIFT)
        inputBuffer.mShift = true;
    if (event->key() == Qt::Key_P)
        inputBuffer.mPlay = !inputBuffer.mPlay;
    if (event->key() == Qt::Key_Escape)
        QCoreApplication::quit();
}

void InputSystem::keyRelease(QKeyEvent *event)
{
    if (event->key() == Qt::Key_W)
        inputBuffer.mForward = 0.0;
    if (event->key() == Qt::Key_S)
        inputBuffer.mForward = 0.0;
    if (event->key() == Qt::Key_D)
        inputBuffer.mRight = 0.0;
    if (event->key() == Qt::Key_A)
        inputBuffer.mRight = 0.0;
    if (event->key() == Qt::Key_Space)
        inputBuffer.mSpace = false;
    if (event->key() == Qt::CTRL)
        inputBuffer.mCtrl = false;
    if (event->key() == Qt::SHIFT)
        inputBuffer.mShift = false;
}

void InputSystem::mousePress(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
        inputBuffer.mRightMouseBtn = true;
    if (event->button() == Qt::LeftButton)
        inputBuffer.mLeftMouseBtn = true;
    if (event->button() == Qt::MiddleButton)
        inputBuffer.mMiddleMouseBtn = true;
}

void InputSystem::mouseRelease(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
        inputBuffer.mRightMouseBtn = false;
    if (event->button() == Qt::LeftButton)
        inputBuffer.mLeftMouseBtn = false;
    if (event->button() == Qt::MiddleButton)
        inputBuffer.mMiddleMouseBtn = false;
}

void InputSystem::mouseMovement(QMouseEvent *event)
{
    inputBuffer.mMousePosX = event->pos().x();
    inputBuffer.mMousePosY = event->pos().y();
}

void InputSystem::connectGamepad()
{
    if (mGamepad)
    {
        QObject::connect(mGamepad, &QGamepad::axisLeftXChanged, this,
                         [=](){this->updateInputs(inputBuffer.mRight,mGamepad->axisLeftX());});

        QObject::connect(mGamepad, &QGamepad::axisLeftYChanged, this,
                         [=](){this->updateInputs(inputBuffer.mForward,mGamepad->axisLeftY());});

        QObject::connect(mGamepad, &QGamepad::axisRightXChanged, this,
                         [=](){this->updateInputs(inputBuffer.mYaw,mGamepad->axisRightX());});

        QObject::connect(mGamepad, &QGamepad::axisRightYChanged, this,
                         [=](){this->updateInputs(inputBuffer.mPitch,mGamepad->axisRightY());});

        QObject::connect(mGamepad, &QGamepad::buttonL2Changed, this,
                         [=](){this->updateInputs(inputBuffer.mButtonL2,mGamepad->buttonL2());});

        QObject::connect(mGamepad, &QGamepad::buttonR2Changed, this,
                         [=](){this->updateInputs(inputBuffer.mButtonR2,mGamepad->buttonR2());});

        QObject::connect(mGamepad, &QGamepad::buttonAChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonA,mGamepad->buttonA());});

        QObject::connect(mGamepad, &QGamepad::buttonBChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonB,mGamepad->buttonB());});

        QObject::connect(mGamepad, &QGamepad::buttonXChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonX,mGamepad->buttonX());});

        QObject::connect(mGamepad, &QGamepad::buttonYChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonY,mGamepad->buttonY());});

        QObject::connect(mGamepad, &QGamepad::buttonL1Changed, this,
                         [=](){this->updateInputs(inputBuffer.mButtonL1,mGamepad->buttonL1());});

        QObject::connect(mGamepad, &QGamepad::buttonL3Changed, this,
                         [=](){this->updateInputs(inputBuffer.mButtonL3,mGamepad->buttonL3());});

        QObject::connect(mGamepad, &QGamepad::buttonR1Changed, this,
                         [=](){this->updateInputs(inputBuffer.mButtonR1,mGamepad->buttonR1());});

        QObject::connect(mGamepad, &QGamepad::buttonR3Changed, this,
                         [=](){this->updateInputs(inputBuffer.mButtonR3,mGamepad->buttonR3());});

        QObject::connect(mGamepad, &QGamepad::buttonRightChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonRight,mGamepad->buttonRight());});

        QObject::connect(mGamepad, &QGamepad::buttonLeftChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonLeft,mGamepad->buttonLeft());});

        QObject::connect(mGamepad, &QGamepad::buttonUpChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonUp,mGamepad->buttonUp());});

        QObject::connect(mGamepad, &QGamepad::buttonDownChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonDown,mGamepad->buttonDown());});

        QObject::connect(mGamepad, &QGamepad::buttonStartChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonStart,mGamepad->buttonStart());});

        QObject::connect(mGamepad, &QGamepad::buttonSelectChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonSelect,mGamepad->buttonSelect());});

        QObject::connect(mGamepad, &QGamepad::buttonCenterChanged, this,
                         [=](){this->updateInputs(inputBuffer.mButtonCenter,mGamepad->buttonCenter());});
    }
    else
        qDebug() << "ERROR - Couldnt connect to gamepad";
}

void InputSystem::updateInputs(GLdouble &reciever, const GLdouble &input)
{
    reciever = input;
}

void InputSystem::updateInputs(bool &reciever, const bool &input)
{
    reciever = input;
}
