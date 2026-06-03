#pragma once

#include <vector>
#include "NvCloth/Factory.h"
#include "NvCloth/Cloth.h"
#include "NvClothExt/ClothFabricCooker.h"

class Cloth {
public:
	Cloth(std::vector<physx::PxVec3> points, std::vector<uint32_t> triangles, std::vector<float> invMasses);
	~Cloth();
	nv::cloth::Cloth *Get() const;
	std::vector<uint32_t> GetMeshIndices() const;
	uint32_t GetNumParticles() const;
	physx::PxVec4 *GetCurrentParticles() const;
	void SetPlaneCollisions(std::vector<physx::PxVec4> planes, std::vector<uint32_t> planesIndices);
	void SetDamping(physx::PxVec3 damping);
	void SetDragCoefficient(float dragCoefficient);
	void SetLiftCoefficient(float liftCoefficient);
	void SetWindVelocity(physx::PxVec3 wind);

private:
	nv::cloth::Fabric *fabric;
	nv::cloth::Cloth *cloth;
	std::vector<uint32_t> indices;

};
