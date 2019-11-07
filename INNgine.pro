QT          += core gui widgets gamepad

TEMPLATE    = app
CONFIG      += c++11

TARGET      = INNgine

RC_ICONS = INNgine.ico

INCLUDEPATH +=  ./GSL \
                ./Resources \
                ./Editor \
                ./ECS   \
                ./ECS/Components \
                ./ECS/Systems \
                ./Resources/LAS

#mac {
#    LIBS += -framework OpenAL
#}
#win32 {
#    INCLUDEPATH += $(OPENAL_HOME)\\include\\AL
#    LIBS *= $(OPENAL_HOME)\\libs\\Win32\\OpenAL32.lib
#    # Copy required DLLs to output directory
#    CONFIG(debug, debug|release) {
#        OpenAL32.commands = copy /Y \"$(OPENAL_HOME)\\bin\\Win32\\OpenAL32.dll\" debug
#        OpenAL32.target = debug/OpenAL32.dll

#        QMAKE_EXTRA_TARGETS += OpenAL32
#        PRE_TARGETDEPS += debug/OpenAL32.dll
#    } else:CONFIG(release, debug|release) {
#        OpenAL32.commands = copy /Y \"$(OPENAL_HOME)\\bin\\Win32\\OpenAL32.dll\" release
#        OpenAL32.target = release/OpenAL32.dll

#        QMAKE_EXTRA_TARGETS += OpenAL32
#        PRE_TARGETDEPS += release/OpenAL32.dll release/OpenAL32.dll
#    } else {
#        error(Unknown set of dependencies.)
#    }
#}

SOURCES += main.cpp \
    ECS/Systems/ballsystem.cpp \
    ECS/Systems/npc.cpp \
    GSL/bspline.cpp \
    renderwindow.cpp \
    Resources/shader.cpp \
    Editor/mainwindow.cpp \
    vertex.cpp \
    Resources/octahedron.cpp \
    Resources/texture.cpp \
    transform.cpp \
    material.cpp \
    colormaterial.cpp \
    texturematerial.cpp \
    camera.cpp \
    controlledball.cpp \
    pointlight.cpp \
    lightcolormaterial.cpp \
    GSL/vec4.cpp \
    GSL/vec3.cpp \
    GSL/vec2.cpp \
    GSL/quaternion.cpp \
    GSL/gsl_math.cpp \
    ECS/entity.cpp \
    Resources/meshbase.cpp \
    ECS/Systems/rendersystem.cpp \
    ECS/Systems/systembase.cpp \
    coreengine.cpp \
    GSL/mat2.cpp \
    GSL/mat3.cpp \
    GSL/mat4.cpp \
    ECS/Systems/inputsystem.cpp \
    Resources/meshfactory.cpp \
    Resources/objmesh.cpp \
    Resources/lasmesh.cpp \
    Resources/LAS/lasreader.cpp \
    Resources/LAS/lasreader_alex.cpp \
    ECS/ecsmanager.cpp \
    mousepicker.cpp \
    ECS/Systems/camerasystem.cpp \
#    ECS/Systems/audiosystem.cpp \
#    ECS/Systems/wavfilehandler.cpp \
#    ECS/Components/soundcomponent.cpp \
    ECS/Systems/movementsystem.cpp \
    normalmaterial.cpp \
    ECS/Systems/collisionsystem.cpp \
    ECS/Components/controlledballscript.cpp \
    ECS/Systems/scriptsystem.cpp \
    ECS/Components/componentbase.cpp \
    ECS/Systems/physicssystem.cpp \
    linedebug.cpp \
    directionallight.cpp

HEADERS += \
    ECS/Systems/ballsystem.h \
    ECS/Systems/npc.h \
    GSL/bspline.h \
    renderwindow.h \
    Resources/shader.h \
    Editor/mainwindow.h \
    vertex.h \
    Resources/octahedron.h \
    Resources/texture.h \
    transform.h \
    constants.h \
    material.h \
    colormaterial.h \
    texturematerial.h \
    camera.h \
    controlledball.h \
    pointlight.h \
    lightcolormaterial.h \
    GSL/vec4.h \
    GSL/vec3.h \
    GSL/vec2.h \
    GSL/math_constants.h \
    GSL/gsl_math.h \
    GSL/quaternion.h \
    ECS/Components/componentbase.h \
    ECS/entity.h \
    gltypes.h \
    Resources/meshbase.h \
    ECS/Systems/rendersystem.h \
    ECS/Systems/systembase.h \
    coreengine.h \
    GSL/mat2.h \
    GSL/mat3.h \
    GSL/mat4.h \
    ECS/Systems/inputsystem.h \
    Resources/meshfactory.h \
    Resources/objmesh.h \
    Resources/lasmesh.h \
    Resources/primitives.h \
    Resources/LAS/lasreader.h \
    Resources/LAS/headerstock.h \
    Resources/LAS/lasreader_alex.h \
    ECS/ecsmanager.h \
    mousepicker.h \
    ECS/Systems/camerasystem.h \
#    ECS/Systems/audiosystem.h \
#    ECS/Systems/wavfilehandler.h \
#    ECS/Components/soundcomponent.h \
    ECS/Systems/movementsystem.h \
    normalmaterial.h \
    ECS/Systems/collisionsystem.h \
    ECS/Components/controlledballscript.h \
    ECS/Systems/scriptsystem.h \
    ECS/Systems/physicssystem.h \
    linedebug.h \
    directionallight.h

FORMS += \
    mainwindow.ui

DISTFILES += \
    plainfragment.frag \
    plainvertex.vert \
    textureshader.frag \
    textureshader.vert \
    lightcolor.vert \
    lightcolor.frag \
    lightnormal.frag \
    lightnormal.vert

RESOURCES += \
    programicons.qrc
