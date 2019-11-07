#ifndef COMPONENTBASE_H
#define COMPONENTBASE_H

#include "gltypes.h"
#include "transform.h"
#include "vec3.h"
#include "quaternion.h"
#include "bspline.h"

class Material;
class Vertex;
class MeshBase;
struct TransformComponent;
struct MeshComponent;
struct MaterialComponent;
struct InputComponent;
struct ScriptComponent;
struct RigidBodyComponent;
struct MovementComponent;
struct ColliderComponent;
struct BoxColliderComponent;
struct SphereColliderComponent;
struct CyllinderColliderComponent;
struct CapsuleColliderComponent;
struct MeshColliderComponent;
struct CameraComponent;
struct NPCComponent;
struct TrophyColliderComponent;

static GLuint ID{0};

enum Shapes
{
    BOX,
    SPHERE,
    CYLLINDER,
    CAPSULE
};


struct plane
{
    gsl::Vec3 position;
    gsl::Vec3 normal;
};

struct Triangle
{
    Triangle(){}
    Triangle(GLint i1, GLint i2, GLint i3) : i{i1}, j{i2}, k{i3}
    {

    }

    Triangle(GLint i1, GLint i2, GLint i3, GLint _e1, GLint _e2, GLint _e3) : i{i1}, j{i2}, k{i3}, e1{_e1}, e2{_e2}, e3{_e3}
    {

    }

    GLint i{-1}, j{-1}, k{-1}; //Index to vertices
    GLint e1{-1}, e2{-1}, e3{-1}; //Index to next triangle (-1 if no triangle)
};

struct ComponentBase
{
    public:
        ComponentBase()
        {
            mComponentID = ID;
            ID++;
        }
        virtual ~ComponentBase(){}

        GLint componentType{0};
        GLuint ownerEntityID{0};
        GLuint mComponentID;

        TransformComponent* getFirstTransformComponent();
        MeshComponent* getFirstMeshComponent();
        MaterialComponent* getFirstMaterialComponent();
        InputComponent* getFirstInputComponent();
        ScriptComponent* getFirstScriptComponent();
        RigidBodyComponent* getFirstRigidbodyComponent();
        MovementComponent* getFirstMovementComponent();
        ColliderComponent* getFirstColliderComponent();
        BoxColliderComponent* getFirstBoxColliderComponent();
        SphereColliderComponent* getFirstSphereColliderComponent();
        CyllinderColliderComponent* getFirstCyllinderColliderComponent();
        CapsuleColliderComponent* getFirstCapsuleColliderComponent();
        MeshColliderComponent* getFirstMeshColliderComponent();
        CameraComponent* getFirstCameraComponent();
        NPCComponent* getFirstNPCComponent();
        TrophyColliderComponent* getFirstTrophyColliderComponent();
};

struct NPCComponent : ComponentBase
{
    MeshBase *mData{nullptr};
    float time{0.f};
    float speed{0.1};

    BSpline path;
    bool direction = false;
    gsl::Vec3 lastPosition;

    int triangleLastFrame{0};
};

struct MeshComponent : ComponentBase
{
    MeshBase *mData{nullptr};       //Gives the posibility to change mesh data or set up a new VAO for different material
    GLenum mDrawType{0};
    GLuint mNumberOfVertices{0};
    GLuint mNumberOfIndices{0};
    GLuint mVAO{0};
    bool mDrawMesh{true};
    bool mDrawBoundingBox{true};
    GLuint mOBB_VAO{0};
};

struct ScriptComponent : ComponentBase
{
    virtual void update(GLfloat){}
    virtual void fixedUpdate(GLfloat){}
};

struct MaterialComponent : ComponentBase
{
    Material *mMaterial{nullptr};   //Should probably just be an int into the Material array.
};

struct TransformComponent : ComponentBase
{
    Transform mTransform;
};

struct RigidBodyComponent : ComponentBase
{
    GLfloat angularDrag;
    gsl::Vec3 angularVelocity;
    gsl::Vec3 totalAngularForce;
    gsl::Vec3 centerOfMass;

    bool detectCollisions{true};
    GLfloat drag;
    bool freezeRotation{false};

    bool isKinematic{true};
    GLfloat mass{1.f};
    GLfloat maxAngularVelocity;

    gsl::Vec3 position;
    gsl::Quaternion rotation;

    bool useGravity{true};
    gsl::Vec3 velocity;
    gsl::Vec3 totalForceApplied;
    gsl::Vec3 worldCenterOfMass;
    bool sleepOneFrame{false};

    //Methods
    void addForce(const gsl::Vec3 &force);
    void addForceAtLocation(const gsl::Vec3 &force, const gsl::Vec3 &location);
    void addRelativeForce(const gsl::Vec3 &force);
    void addRelavtiveTorque(const gsl::Vec3 &torque);
    void addTorque(const gsl::Vec3 &torque);

    void movePosition(const gsl::Vec3 &pos);
    void moveRotation(const gsl::Quaternion &rot);
    void resetCenterOfMass();

    void Sleep();
    void WakeUp();
    void SweepTest();
    void SweepTestAll();
};

struct InputComponent : ComponentBase
{
    //Keyboard/mouse specific
    GLdouble mMousePosX{0.0}, mMousePosY{0.0};
    bool mMiddleMouseBtn{false}, mLeftMouseBtn{false}, mRightMouseBtn{false};
    bool mSpace{false}, mCtrl{false}, mShift{false}, mPlay{false};

    //Keyboard and gamepad
    GLdouble mForward{0.0}, mRight{0.0}, mPitch{0.0}, mYaw{0.0};

    //Gamepad specific
    GLdouble mButtonL2{0.0}, mButtonR2{0.0};
    bool mButtonL1{false}, mButtonL3{false}, mButtonR1{false}, mButtonR3{false};
    bool mButtonA{false}, mButtonB{false}, mButtonX{false}, mButtonY{false};
    bool mButtonUp{false}, mButtonDown{false}, mButtonLeft{false}, mButtonRight{false};
    bool mButtonStart{false}, mButtonSelect{false}, mButtonCenter{false};
};

struct MovementComponent : ComponentBase
{
    gsl::Vec3 forwardVector{gsl::Vec3(1.f, 0.f, 0.f)};
    GLfloat currentSpeed{};
    GLfloat maxSpeed{30.f};
};

struct ColliderComponent : ComponentBase
{
    void *onInside{nullptr};
};

struct BoxColliderComponent : ColliderComponent
{
    plane f1, f2, f3, f4, f5, f6;
    gsl::Vec3 centerPosition;
    bool bSpline;
};

struct SphereColliderComponent : ColliderComponent
{
    GLfloat radius;
    gsl::Vec3 centerPosition;
    bool touching{false};
};

struct CyllinderColliderComponent : ColliderComponent
{
    GLfloat radius, height;
    gsl::Vec3 centerPosition;
};

struct CapsuleColliderComponent : ColliderComponent
{
    GLfloat radius, height;
    gsl::Vec3 centerPosition;
};

struct MeshColliderComponent : ColliderComponent
{
    MeshBase *mData{nullptr}; //Pointer to meshdata so we can access mVertices
    //We need to have a per-triangle collision
    Triangle *triangles;
    size_t numTriangles;

    gsl::Mat4 modelMatrix;
};


struct TrophyColliderComponent : ColliderComponent
{
    GLfloat pickUpRadius;
    gsl::Vec3 centerPosition;
};

struct CameraComponent : public ComponentBase
{
    Transform cameraTransform; // transform of parent

    // It's good if theese are on the stack
    gsl::Mat4 mViewMatrix;

    // Only update perspective matrix when this is toggled
    bool persValuesChanged{true};
    gsl::Mat4 mPerspectiveMatrix;

    bool followEntity{false};
    bool inheritRotationX{false}; // Should the camera have the same rotation along x as the parent?
    bool inheritRotationY{false}; // Should the camera have the same rotation along y as the parent?
    bool inheritRotationZ{false}; // Should the camera have the same rotation along z as the parent?
    bool orbitCamera{false};    // Should the camera orbit?
    GLuint mParentTransformID{0};
    GLuint mFollowEntityID{0};

    GLfloat mNearPlane{0.1f};
    GLfloat mFarPlane{1000.f};
    GLfloat mFieldOfView{45.f};
    GLfloat mAspectRatio{1.33f};

    GLfloat mDistance = 10.f; // If orbit, what distance?

    gsl::Vec3 mPositionOffset; // Used only in orbit mode
    gsl::Vec3 mTarget{0.f, 0.f, 0.f};       //What to look at
    gsl::Vec3 mDirection{0.f, 0.f, -1.f};    //Direction to look
};

#endif // COMPONENTBASE_H
