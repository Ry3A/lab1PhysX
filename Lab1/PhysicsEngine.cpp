#include "PhysicsEngine.h"

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

	pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT
		| physx::PxPairFlag::eDETECT_DISCRETE_CONTACT
		| physx::PxPairFlag::eDETECT_CCD_CONTACT;

	return physx::PxFilterFlag::eDEFAULT;
}

#define SAFE_RELEASE(obj) {	\
	if (obj) {				\
		obj->release();		\
		obj = nullptr;		\
	}						\
}

#define IS_FLOATS_EQUAL(f1, f2) (fabsf((f1) - (f2)) < 1e-5f)

PhysicsEngine::PhysicsEngine() {
	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocatorCallback, errorCallback);

#ifdef USE_PVD
	pvd = physx::PxCreatePvd(*foundation);
	transport = physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10000);
	if (!pvd || !transport || !pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL)) {
		std::cout << "[WARNING] Could not initialize or connect pvd\n";
	}

	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd);
	PxInitExtensions(*physics, pvd);
#else
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), false);
#endif

	physx::PxSceneDesc sceneDesc = physx::PxSceneDesc(physics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	dispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = dispatcher;
	sceneDesc.filterShader = CustomFilterShader;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_CCD;

	scene = physics->createScene(sceneDesc);

#ifdef USE_PVD
	if (pvd && pvd->isConnected()) {
		physx::PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
		if (pvdClient) {
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}
	}
#endif
}

PhysicsEngine::~PhysicsEngine()
{
	// Ничего не fetchResults при завершении
	// и не removeActor + release по отдельности

	if (scene)
	{
		std::vector<physx::PxRigidActor*> actors = GetActors();
		for (physx::PxRigidActor* actor : actors)
		{
			if (actor)
			{
				actor->release();
			}
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

void PhysicsEngine::SetGravity(physx::PxVec3 gravity) {
	scene->setGravity(gravity);
}

void PhysicsEngine::Simulate(float elapsedTime) {
	scene->simulate(elapsedTime);
	scene->fetchResults(true);
}

physx::PxMaterial* PhysicsEngine::CreateMaterial(float staticFriction, float dynamicFriction, float restitution) {
	physx::PxMaterial* material = physics->createMaterial(staticFriction, dynamicFriction, restitution);
	return material;
}

physx::PxMaterial* PhysicsEngine::GetMaterial(float staticFriction, float dynamicFriction, float restitution) {
	uint32_t materialsNum = physics->getNbMaterials();
	if (materialsNum > 0) {
		std::vector<physx::PxMaterial*> materials(materialsNum);
		physics->getMaterials(reinterpret_cast<physx::PxMaterial**>(materials.data()), materialsNum);
		for (auto* material : materials) {
			if (IS_FLOATS_EQUAL(material->getStaticFriction(), staticFriction)
				&& IS_FLOATS_EQUAL(material->getDynamicFriction(), dynamicFriction)
				&& IS_FLOATS_EQUAL(material->getRestitution(), restitution)) {
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
	physx::PxShapeFlags shapeFlags
) {
	physx::PxBoxGeometry geometry = physx::PxBoxGeometry(size / 2.0f);
	physx::PxShape* shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);
	return shape;
}

physx::PxShape* PhysicsEngine::CreateBoxShape(
	physx::PxVec3 size,
	physx::PxVec3 position,
	physx::PxQuat rotation,
	physx::PxMaterial* material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxShape* shape = CreateBoxShape(size, material, isExclusive, shapeFlags);
	shape->setLocalPose(physx::PxTransform(position, rotation));
	return shape;
}

physx::PxShape* PhysicsEngine::CreateSphereShape(
	float radius,
	physx::PxMaterial* material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxSphereGeometry geometry = physx::PxSphereGeometry(radius);
	physx::PxShape* shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);
	return shape;
}

physx::PxShape* PhysicsEngine::CreateSphereShape(
	float radius,
	physx::PxVec3 position,
	physx::PxMaterial* material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxShape* shape = CreateSphereShape(radius, material, isExclusive, shapeFlags);
	shape->setLocalPose(physx::PxTransform(position));
	return shape;
}

physx::PxShape* PhysicsEngine::CreateCapsuleShape(
	float radius,
	float size,
	physx::PxMaterial* material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxCapsuleGeometry geometry = physx::PxCapsuleGeometry(radius, size / 2.0f);
	physx::PxShape* shape = physics->createShape(geometry, *material, isExclusive, shapeFlags);
	return shape;
}

physx::PxShape* PhysicsEngine::CreateCapsuleShape(
	float radius,
	float size,
	physx::PxVec3 position,
	physx::PxQuat rotation,
	physx::PxMaterial* material,
	bool isExclusive,
	physx::PxShapeFlags shapeFlags
) {
	physx::PxShape* shape = CreateCapsuleShape(radius, size, material, isExclusive, shapeFlags);
	shape->setLocalPose(physx::PxTransform(position, rotation));
	return shape;
}

float PhysicsEngine::GetBoxVolume(physx::PxVec3 size) {
	return size.x * size.y * size.z;
}

float PhysicsEngine::GetSphereVolume(float radius) {
	return 1.3333f * physx::PxPi * radius * radius * radius;
}

float PhysicsEngine::GetCapsuleVolume(float radius, float size) {
	return physx::PxPi * radius * radius * (1.3333f * radius + size);
}

physx::PxRigidStatic* PhysicsEngine::AddGround(physx::PxVec3 normal, float distance, physx::PxMaterial* material) {
	physx::PxPlane plane = physx::PxPlane(
		IS_FLOATS_EQUAL(normal.magnitudeSquared(), 1.0f) ? normal : normal.getNormalized(),
		distance
	);
	physx::PxRigidStatic* groundPlane = PxCreatePlane(*physics, plane, *material);
	scene->addActor(*groundPlane);
	return groundPlane;
}

physx::PxRigidStatic* PhysicsEngine::AddStaticActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation) {
	physx::PxRigidStatic* actor = physics->createRigidStatic(physx::PxTransform(position, rotation));
	actor->attachShape(*shape);
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidDynamic* PhysicsEngine::AddDynamicActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation, float density) {
	physx::PxRigidDynamic* actor = physics->createRigidDynamic(physx::PxTransform(position, rotation));
	actor->attachShape(*shape);
	physx::PxRigidBodyExt::updateMassAndInertia(*actor, density);
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidStatic* PhysicsEngine::AddStaticActor(std::vector<physx::PxShape*> shapes, physx::PxVec3 position, physx::PxQuat rotation) {
	physx::PxRigidStatic* actor = physics->createRigidStatic(physx::PxTransform(position, rotation));
	for (auto* shape : shapes) {
		actor->attachShape(*shape);
	}
	scene->addActor(*actor);
	return actor;
}

physx::PxRigidDynamic* PhysicsEngine::AddDynamicActor(std::vector<physx::PxShape*> shapes, physx::PxVec3 position, physx::PxQuat rotation, float density) {
	physx::PxRigidDynamic* actor = physics->createRigidDynamic(physx::PxTransform(position, rotation));
	for (auto* shape : shapes) {
		actor->attachShape(*shape);
	}
	physx::PxRigidBodyExt::updateMassAndInertia(*actor, density);
	scene->addActor(*actor);
	return actor;
}

std::vector<physx::PxRigidActor*> PhysicsEngine::GetActors(physx::PxActorTypeFlags types) {
	if (!scene)
		return {};

	uint32_t actorsNum = scene->getNbActors(types);
	std::vector<physx::PxRigidActor*> actors(actorsNum);

	if (actorsNum > 0)
		scene->getActors(types, reinterpret_cast<physx::PxActor**>(actors.data()), actorsNum);

	return actors;
}

void PhysicsEngine::RemoveActor(physx::PxActor* actor) {
	if (!scene || !actor)
		return;

	scene->removeActor(*actor);
}
