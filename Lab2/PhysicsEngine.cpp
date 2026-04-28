#include "PhysicsEngine.h"

#define SAFE_RELEASE(obj) { \
    if (obj) {              \
        obj->release();     \
        obj = nullptr;      \
    }                       \
}

static bool FloatEquals(float a, float b)
{
    return std::fabs(a - b) < 1e-5f;
}

static physx::PxFilterFlags CustomFilterShader(
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1,
    physx::PxPairFlags& pairFlags,
    const void* constantBlock,
    physx::PxU32 constantBlockSize)
{
    PX_UNUSED(attributes0);
    PX_UNUSED(filterData0);
    PX_UNUSED(attributes1);
    PX_UNUSED(filterData1);
    PX_UNUSED(constantBlock);
    PX_UNUSED(constantBlockSize);

    pairFlags =
        physx::PxPairFlag::eCONTACT_DEFAULT |
        physx::PxPairFlag::eDETECT_DISCRETE_CONTACT |
        physx::PxPairFlag::eDETECT_CCD_CONTACT;

    return physx::PxFilterFlag::eDEFAULT;
}

PhysicsEngine::PhysicsEngine()
{
    foundation = PxCreateFoundation(
        PX_PHYSICS_VERSION,
        allocatorCallback,
        errorCallback
    );

#ifdef USE_PVD
    pvd = physx::PxCreatePvd(*foundation);
    transport = physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10000);

    if (!pvd || !transport || !pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL))
        std::cout << "[WARNING] PVD connection failed\n";

    physics = PxCreatePhysics(
        PX_PHYSICS_VERSION,
        *foundation,
        physx::PxTolerancesScale(),
        true,
        pvd
    );

    PxInitExtensions(*physics, pvd);
#else
    physics = PxCreatePhysics(
        PX_PHYSICS_VERSION,
        *foundation,
        physx::PxTolerancesScale(),
        false
    );
#endif

    physx::PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);

    dispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = dispatcher;
    sceneDesc.filterShader = CustomFilterShader;
    sceneDesc.flags |= physx::PxSceneFlag::eENABLE_CCD;

    scene = physics->createScene(sceneDesc);

#ifdef USE_PVD
    if (pvd && pvd->isConnected())
    {
        physx::PxPvdSceneClient* pvdClient = scene->getScenePvdClient();

        if (pvdClient)
        {
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
        }
    }
#endif
}

PhysicsEngine::~PhysicsEngine()
{
    if (scene)
    {
        std::vector<physx::PxRigidActor*> actors = GetActors();

        for (physx::PxRigidActor* actor : actors)
        {
            if (actor)
                actor->release();
        }
    }

    SAFE_RELEASE(scene);
    SAFE_RELEASE(dispatcher);

#ifdef USE_PVD
    if (pvd)
    {
        if (pvd->isConnected())
            pvd->disconnect();

        SAFE_RELEASE(pvd);
    }

    SAFE_RELEASE(transport);
    PxCloseExtensions();
#endif

    SAFE_RELEASE(physics);
    SAFE_RELEASE(foundation);
}

void PhysicsEngine::Simulate(float dt)
{
    scene->simulate(dt);
    scene->fetchResults(true);
}

void PhysicsEngine::RemoveActor(physx::PxActor* actor)
{
    if (!scene || !actor)
        return;

    scene->removeActor(*actor);
}

physx::PxPhysics* PhysicsEngine::GetPhysics()
{
    return physics;
}

physx::PxScene* PhysicsEngine::GetScene()
{
    return scene;
}

physx::PxMaterial* PhysicsEngine::CreateMaterial(
    float staticFriction,
    float dynamicFriction,
    float restitution)
{
    return physics->createMaterial(
        staticFriction,
        dynamicFriction,
        restitution
    );
}

physx::PxMaterial* PhysicsEngine::GetMaterial(
    float staticFriction,
    float dynamicFriction,
    float restitution)
{
    physx::PxU32 count = physics->getNbMaterials();

    if (count > 0)
    {
        std::vector<physx::PxMaterial*> materials(count);
        physics->getMaterials(materials.data(), count);

        for (physx::PxMaterial* material : materials)
        {
            if (
                FloatEquals(material->getStaticFriction(), staticFriction) &&
                FloatEquals(material->getDynamicFriction(), dynamicFriction) &&
                FloatEquals(material->getRestitution(), restitution)
                )
            {
                return material;
            }
        }
    }

    return CreateMaterial(staticFriction, dynamicFriction, restitution);
}

physx::PxShape* PhysicsEngine::CreateBoxShape(
    physx::PxVec3 size,
    physx::PxMaterial* material,
    bool isExclusive,
    physx::PxShapeFlags flags)
{
    physx::PxBoxGeometry geometry(size * 0.5f);
    return physics->createShape(geometry, *material, isExclusive, flags);
}

physx::PxShape* PhysicsEngine::CreateSphereShape(
    float radius,
    physx::PxMaterial* material,
    bool isExclusive,
    physx::PxShapeFlags flags)
{
    physx::PxSphereGeometry geometry(radius);
    return physics->createShape(geometry, *material, isExclusive, flags);
}

physx::PxShape* PhysicsEngine::CreateCapsuleShape(
    float radius,
    float size,
    physx::PxMaterial* material,
    bool isExclusive,
    physx::PxShapeFlags flags)
{
    physx::PxCapsuleGeometry geometry(radius, size * 0.5f);
    return physics->createShape(geometry, *material, isExclusive, flags);
}

physx::PxRigidStatic* PhysicsEngine::AddGround(
    physx::PxVec3 normal,
    float distance,
    physx::PxMaterial* material)
{
    if (normal.magnitudeSquared() < 0.0001f)
        normal = physx::PxVec3(0.0f, 1.0f, 0.0f);

    normal.normalize();

    physx::PxPlane plane(normal, distance);
    physx::PxRigidStatic* ground = PxCreatePlane(*physics, plane, *material);
    scene->addActor(*ground);

    return ground;
}

physx::PxRigidStatic* PhysicsEngine::AddStaticActor(
    physx::PxShape* shape,
    physx::PxVec3 position,
    physx::PxQuat rotation)
{
    physx::PxRigidStatic* actor =
        physics->createRigidStatic(physx::PxTransform(position, rotation));

    actor->attachShape(*shape);
    scene->addActor(*actor);

    return actor;
}

physx::PxRigidDynamic* PhysicsEngine::AddDynamicActor(
    physx::PxShape* shape,
    physx::PxVec3 position,
    physx::PxQuat rotation,
    float density)
{
    physx::PxRigidDynamic* actor =
        physics->createRigidDynamic(physx::PxTransform(position, rotation));

    actor->attachShape(*shape);
    physx::PxRigidBodyExt::updateMassAndInertia(*actor, density);
    scene->addActor(*actor);

    return actor;
}

std::vector<physx::PxRigidActor*> PhysicsEngine::GetActors(
    physx::PxActorTypeFlags types)
{
    if (!scene)
        return {};

    physx::PxU32 count = scene->getNbActors(types);
    std::vector<physx::PxRigidActor*> actors(count);

    if (count > 0)
    {
        scene->getActors(
            types,
            reinterpret_cast<physx::PxActor**>(actors.data()),
            count
        );
    }

    return actors;
}