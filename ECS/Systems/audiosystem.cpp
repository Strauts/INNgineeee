#include "audiosystem.h"
#include <iostream>

AudioSystem* AudioSystem::mInstance = nullptr;

AudioSystem::AudioSystem() : SystemBase()
{
}

AudioSystem *AudioSystem::getInstance()          ///< Get pointer to singleton instance.
{
    if (!mInstance)
        mInstance = new AudioSystem();
    return mInstance;
}

void AudioSystem::init()
{
    std::cout << "Intializing OpenAL!\n";
    mDevice = alcOpenDevice(nullptr);
    if (mDevice)
    {
        mContext = alcCreateContext(mDevice, nullptr);
        alcMakeContextCurrent(mContext);
    }

    // Generate buffers
    alGetError();

    if (!mDevice)
    {
        std::cout << "Device not made!\n";
    }
    else
        std::cout << "Intialization complete!\n";

    //Start listing of found sound devices:
    //Not jet implemented
    //ALDeviceList *pDeviceList = NULL;
    //ALCcontext *pContext = NULL;
    //ALCdevice *pDevice = NULL;
    //ALint i;	//will hold the number of the preferred device
    //ALboolean bReturn = AL_FALSE;
}

void AudioSystem::cleanUp()
{
    mContext = alcGetCurrentContext();
    mDevice = alcGetContextsDevice(mContext);
    alcMakeContextCurrent(nullptr);
    alcDestroyContext(mContext);
    alcCloseDevice(mDevice);
}

SoundComponent* AudioSystem::createSource(std::string name, Vec3 pos, std::string filePath, bool loop, float gain)
{
    SoundComponent* tempPtr = new SoundComponent(name, loop, gain);
    tempPtr->setPosition(pos);
    if (filePath != "")
        tempPtr->loadWave(filePath);
    return tempPtr;
}

void AudioSystem::updateListener(Vec3 pos, Vec3 vel, Vec3 dir, Vec3 up)
{
    ALfloat posVec[3];
    ALfloat velVec[3];
    ALfloat headVec[6];
    posVec[0] = pos.getX();
    posVec[1] = pos.getY();
    posVec[2] = pos.getZ();
    velVec[0] = vel.getX();
    velVec[1] = vel.getY();
    velVec[2] = vel.getZ();
    headVec[0] = dir.getX();
    headVec[1] = dir.getY();
    headVec[2] = dir.getZ();
    headVec[3] = up.getX();
    headVec[4] = up.getY();
    headVec[5] = up.getZ();
    alListenerfv(AL_POSITION, posVec);
    alListenerfv(AL_VELOCITY, velVec);
    alListenerfv(AL_ORIENTATION, headVec);
}

bool AudioSystem::checkError()
{
    switch (alGetError())
    {
    case AL_NO_ERROR:
        break;
    case AL_INVALID_NAME:
        std::cout << "Invalid name!\n";
        return false;
    case AL_INVALID_ENUM:
        std::cout << "Invalid enum!\n";
        return false;
    case AL_INVALID_VALUE:
        std::cout << "Invalid value!\n";
        return false;
    case AL_INVALID_OPERATION:
        std::cout << "Invalid operation!\n";
        return false;
    case AL_OUT_OF_MEMORY:
        std::cout << "Out of memory!\n";
        return false;
    default: break;
    }
    return true;
}
