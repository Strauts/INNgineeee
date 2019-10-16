#ifndef AUDIOSYSTEM_H
#define AUDIOSYSTEM_H

#ifdef _WIN32
#include <al.h>
#include <alc.h>
#endif
#ifdef __APPLE__
#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#endif

#include <QOpenGLFunctions_4_1_Core>
#include <string>
#include <vector>
#include "systembase.h"
#include "soundcomponent.h"
#include "vec3.h"

class AudioSystem : public SystemBase, public QOpenGLFunctions_4_1_Core
{
public:
    AudioSystem();

    struct SoundListener
    {
        Vec3 pos;
        Vec3 vel;
        Vec3 dir;
        Vec3 up;
        SoundListener()
        {
            pos = Vec3(0.0f, 0.0f, 0.0f);
            vel = Vec3(0.0f, 0.0f, 0.0f);
            dir = Vec3(0.0f, 0.0f, -1.0f);
            up = Vec3(0.0f, 1.0f, 0.0f);
        }
    };

    static AudioSystem *getInstance();

    /// Initialises OpenAL.
    virtual void init() override;
    /// Cleans up and shuts down OpenAl.
    void cleanUp();

    SoundComponent* createSource(std::string name, Vec3 pos, std::string filePath = "", bool loop = false, float gain = 1.0);
    void updateListener(Vec3 pos, Vec3 vel, Vec3 dir, Vec3 up);

private:
    AudioSystem(AudioSystem const&);      ///< Private copy constructor.
    void operator=(AudioSystem const&);    ///< Private class assignment operator.
    static AudioSystem* mInstance;         ///< Singleton instance pointer.
    /// Debug function.
    /**
        Checks for and outputs OpenAL errors.
    **/
    bool checkError();

    ALCdevice* mDevice{nullptr};                 ///< Pointer to the ALC Device.
    ALCcontext* mContext{nullptr};               ///< Pointer to the ALC Context.


};

#endif // AUDIOSYSTEM_H
