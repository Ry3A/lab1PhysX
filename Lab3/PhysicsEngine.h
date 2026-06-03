#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include "PxPhysicsAPI.h"
#include "NvCloth/Factory.h"
#include "PxPhysicsAPI.h"
#include "NvCloth/Callbacks.h"


#ifdef _DEBUG
//#define USE_PVD
#define PVD_HOST "127.0.0.1"
#endif

class PhysicsEngine {
public:
	PhysicsEngine();
	~PhysicsEngine();

	
	void SetGravity(physx::PxVec3 gravity);
	void Simulate(float elapsedTime);

	physx::PxMaterial* CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
	physx::PxMaterial* GetMaterial(float staticFriction, float dynamicFriction, float restitution);
	physx::PxScene* GetScene() const;
	nv::cloth::Factory* GetClothFactory() const;

	physx::PxShape* CreateBoxShape(
		physx::PxVec3 size,
		physx::PxMaterial* material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);

	physx::PxShape* CreateBoxShape(
		physx::PxVec3 size,
		physx::PxVec3 position,
		physx::PxQuat rotation,
		physx::PxMaterial* material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);

	physx::PxShape* CreateSphereShape(
		float radius,
		physx::PxMaterial* material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);

	physx::PxShape* CreateSphereShape(
		float radius,
		physx::PxVec3 position,
		physx::PxMaterial* material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);

	physx::PxShape* CreateCapsuleShape(
		float radius,
		float size,
		physx::PxMaterial* material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);

	physx::PxShape* CreateCapsuleShape(
		float radius,
		float size,
		physx::PxVec3 position,
		physx::PxQuat rotation,
		physx::PxMaterial* material,
		bool isExclusive = false,
		physx::PxShapeFlags shapeFlags = physx::PxShapeFlag::eVISUALIZATION | physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE
	);

	static float GetBoxVolume(physx::PxVec3 size);
	static float GetSphereVolume(float radius);
	static float GetCapsuleVolume(float radius, float size);

	physx::PxRigidStatic* AddGround(physx::PxVec3 normal, float distance, physx::PxMaterial* material);
	physx::PxRigidStatic* AddStaticActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation);
	physx::PxRigidDynamic* AddDynamicActor(physx::PxShape* shape, physx::PxVec3 position, physx::PxQuat rotation, float density);

	physx::PxRigidStatic* AddStaticActor(std::vector<physx::PxShape*> shapes, physx::PxVec3 position, physx::PxQuat rotation);
	physx::PxRigidDynamic* AddDynamicActor(std::vector<physx::PxShape*> shapes, physx::PxVec3 position, physx::PxQuat rotation, float density);

	std::vector<physx::PxRigidActor*> GetActors(
		physx::PxActorTypeFlags types = physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC
	);

	void RemoveActor(physx::PxActor* actor);

private:
	class NvClothAssertHandler : public nv::cloth::PxAssertHandler
	{
	public:
		void operator()(const char* exp, const char* file, int line, bool& ignore) override
		{
			ignore = false;

			std::cout
				<< "[NvCloth Assert] "
				<< exp
				<< " in "
				<< file
				<< ":"
				<< line
				<< std::endl;
		}
	};

	NvClothAssertHandler clothAssertHandler;
	nv::cloth::Factory* clothFactory = nullptr;
	physx::PxDefaultAllocator allocatorCallback;
	physx::PxDefaultErrorCallback errorCallback;
	physx::PxFoundation* foundation = nullptr;
	physx::PxPhysics* physics = nullptr;
	physx::PxDefaultCpuDispatcher* dispatcher = nullptr;
#ifdef USE_PVD
	physx::PxPvd* pvd = nullptr;
	physx::PxPvdTransport* transport = nullptr;
#endif
	physx::PxScene* scene = nullptr;

};