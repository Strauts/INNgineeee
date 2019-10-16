#ifndef SOUNDSOURCE_H
#define SOUNDSOURCE_H
#ifdef _WIN32
#include <al.h>
#include <alc.h>
#endif
#ifdef __APPLE__
#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#endif

#include <string>
#include "vec3.h"
#include "entity.h"

using namespace gsl;
/// One sound source.
/**
    Contains one source and one buffer.
    Intended to play a single clip.
    Does not support streaming audio (yet).
**/
struct SoundComponent : public ComponentBase
{
    /// Constructor.
    /**
        Takes two arguments, initialises some variables.
        \param The name of the sound source. Not used.
        \param A boolean to check if sound should loop.
    **/
    SoundComponent(std::string name, bool loop = false, float gain = 1.0);
    ~SoundComponent();

    /// Plays the sound source from the current position.
    void play();
    /// Pauses the sound source at the current position.
    void pause();
    /// Stops the sound source and rewinds to start.
    void stop();

    /// Loads one given WAVE file.
    /**
        Calls the wave loader from the FileHandler class, parses the wave data and buffers it.
        \param The file path relative to execution directory.
    **/
    bool loadWave(std::string filePath);

    void setPosition(Vec3 newPos);             ///< Sets source position from Vector3.
    Vec3 getPosition() {return mPosition;}
    void setVelocity(Vec3 newVel);             ///< Sets source velocity from Vector3.
    Vec3 getVelocity() {return mVelocity;}     ///< Returns source velocity as Vector3.

    std::string mName;          ///< The name of the sound source (Not used).
    ALuint mSource;             ///< The sound source.
    ALuint mBuffer;             ///< The data buffer.
    Vec3 mPosition;    ///< Vector containing source position.
    Vec3 mVelocity;    ///< Vector containing source velocity.
    bool checkError(std::string name);
};
#endif // SOUNDSOURCE_H
