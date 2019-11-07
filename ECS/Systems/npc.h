#ifndef NPC_H
#define NPC_H

#include "systembase.h"
#include "componentbase.h"
#include <vector>

class ECSManager;

class NPC : public SystemBase
{
public:
    NPC();

    void patrol(GLfloat deltatime);
    static NPC getInstance();

private:
    ECSManager* mManager{nullptr};
    static NPC* mInstance;
};

#endif // NPC_H
