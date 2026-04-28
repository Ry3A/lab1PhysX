#pragma once

#include <vector>
#include <iostream>
#include <cmath>

#include "PxPhysicsAPI.h"
#include "extensions/PxExtensionsAPI.h"

#ifdef _DEBUG
// #define USE_PVD
#define PVD_HOST "127.0.0.1"
#endif

class PhysicsEngine
{
public:
    PhysicsEngine();
    ~PhysicsEngine();

    void Simulate(float dt);
    void RemoveActor(physx::PxActor* actor);

    physx::PxPhysics* GetPhysics();
    physx::PxScene* GetScene();

    physx::PxMaterial* CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
    physx::PxMaterial* GetMaterial(float staticFriction, float dynamicFriction, float restitution);

    physx::PxShape* CreateBoxShape(
        physx::PxVec3 size,
        physx::PxMaterial* material,
        bool isExclusive = false,
        physx::PxShapeFlags flags =
        physx::PxShapeFlag::eVISUALIZATION |
        physx::PxShapeFlag::eSCENE_QUERY_SHAPE |
        physx::PxShapeFlag::eSIMULATION_SHAPE
    );

    physx::PxShape* CreateSphereShape(
        float radius,
        physx::PxMaterial* material,
        bool isExclusive = false,
        physx::PxShapeFlags flags =
        physx::PxShapeFlag::eVISUALIZATION |
        physx::PxShapeFlag::eSCENE_QUERY_SHAPE |
        physx::PxShapeFlag::eSIMULATION_SHAPE
    );

    physx::PxShape* CreateCapsuleShape(
        float radius,
        float size,
        physx::PxMaterial* material,
        bool isExclusive = false,
        physx::PxShapeFlags flags =
        physx::PxShapeFlag::eVISUALIZATION |
        physx::PxShapeFlag::eSCENE_QUERY_SHAPE |
        physx::PxShapeFlag::eSIMULATION_SHAPE
    );

    physx::PxRigidStatic* AddGround(
        physx::PxVec3 normal,
        float distance,
        physx::PxMaterial* material
    );

    physx::PxRigidStatic* AddStaticActor(
        physx::PxShape* shape,
        physx::PxVec3 position,
        physx::PxQuat rotation
    );

    physx::PxRigidDynamic* AddDynamicActor(
        physx::PxShape* shape,
        physx::PxVec3 position,
        physx::PxQuat rotation,
        float density
    );

    std::vector<physx::PxRigidActor*> GetActors(
        physx::PxActorTypeFlags types =
        physx::PxActorTypeFlag::eRIGID_STATIC |
        physx::PxActorTypeFlag::eRIGID_DYNAMIC
    );

private:
    physx::PxDefaultAllocator allocatorCallback;
    physx::PxDefaultErrorCallback errorCallback;

    physx::PxFoundation* foundation = nullptr;
    physx::PxPhysics* physics = nullptr;
    physx::PxScene* scene = nullptr;
    physx::PxDefaultCpuDispatcher* dispatcher = nullptr;

#ifdef USE_PVD
    physx::PxPvd* pvd = nullptr;
    physx::PxPvdTransport* transport = nullptr;
#endif
};