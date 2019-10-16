#ifndef CONTROLLEDBALLSCRIPT_H
#define CONTROLLEDBALLSCRIPT_H

#include "componentbase.h"
#include "vec2.h"
#include "gltypes.h"

class ControlledBallScriptComponent : public ScriptComponent
{
    public:
        ControlledBallScriptComponent();
        ~ControlledBallScriptComponent() override;

        void update(GLfloat deltaTime) override;
    private:
        bool lastButtonState = false;

        bool lastMouseSet = false;
        GLfloat mouseSensitivity = 0.6f;
        GLfloat controllerSensitivity = 3.f;
        gsl::Vec2 lastMousePos;
        GLfloat theta{0}, phi{0};
};

#endif // CONTROLLEDBALLSCRIPT_H
