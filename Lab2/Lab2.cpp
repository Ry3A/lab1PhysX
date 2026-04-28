#include <vector>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <windows.h>

#include "PhysicsEngine.h"
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

using namespace physx;

#define SAFE_RELEASE(obj) { \
    if (obj) {              \
        obj->release();     \
        obj = nullptr;      \
    }                       \
}

namespace GameParams
{
    constexpr float FIELD_HALF_SIZE = 18.0f;
    constexpr float WALL_HEIGHT = 2.5f;

    constexpr float ENEMY_RADIUS = 0.45f;
    constexpr float ENEMY_HEIGHT = 2.0f;
    constexpr float ENEMY_DENSITY = 35.0f;

    constexpr float SHOT_MAX_DISTANCE = 80.0f;
    constexpr float SHOT_SPREAD_DEGREES = 2.5f;
    constexpr float SHOT_IMPULSE = 110.0f;
    constexpr float SHOT_VISUAL_TIME = 0.45f;

    constexpr float GRENADE_RADIUS = 0.22f;
    constexpr float GRENADE_DENSITY = 70.0f;
    constexpr float GRENADE_THROW_SPEED = 18.0f;
    constexpr float GRENADE_UP_BOOST = 5.5f;
    constexpr float GRENADE_FUSE_TIME = 2.7f;
    constexpr float GRENADE_EXPLOSION_RADIUS = 5.5f;
    constexpr float GRENADE_MAX_DAMAGE = 100.0f;
    constexpr float GRENADE_MAX_IMPULSE = 320.0f;
    constexpr float EXPLOSION_VISUAL_TIME = 0.55f;

    constexpr float ENEMY_VIEW_RADIUS = 11.0f;
    constexpr float ENEMY_MOVE_SPEED = 3.2f;
    constexpr float COVER_OFFSET = 1.35f;
    constexpr float FLEE_STEP = 7.0f;
}

struct BoxObstacle
{
    PxRigidStatic* actor = nullptr;
    PxVec3 center = PxVec3(0.0f);
    PxVec3 size = PxVec3(0.0f);
};

struct Grenade
{
    PxRigidDynamic* actor = nullptr;
    float fuseLeft = 0.0f;
};

PhysicsEngine* engine = nullptr;
Snippets::Camera* mainCamera = nullptr;

PxMaterial* groundMaterial = nullptr;
PxMaterial* wallMaterial = nullptr;
PxMaterial* enemyMaterial = nullptr;
PxMaterial* grenadeMaterial = nullptr;

PxRigidDynamic* enemyCapsule = nullptr;
PxVec3 playerPosition(0.0f, 3.0f, 14.0f);

std::vector<BoxObstacle> obstacles;
std::vector<Grenade> grenades;

bool enemyRagdollActive = false;
bool enemyHasSeenPlayer = false;
bool enemyMovingToCover = false;

std::vector<PxRigidDynamic*> ragdollBodies;
std::vector<PxJoint*> ragdollJoints;
PxRigidDynamic* ragdollTorso = nullptr;
PxRigidDynamic* ragdollHead = nullptr;
PxRigidDynamic* ragdollLeftArm = nullptr;
PxRigidDynamic* ragdollRightArm = nullptr;
PxRigidDynamic* ragdollLeftLeg = nullptr;
PxRigidDynamic* ragdollRightLeg = nullptr;

bool shotLineVisible = false;
PxVec3 shotLineStart(0.0f);
PxVec3 shotLineEnd(0.0f);
float shotLineTimeLeft = 0.0f;

bool explosionVisible = false;
PxVec3 explosionCenter(0.0f);
float explosionTimeLeft = 0.0f;

void PrintConsoleText(const std::wstring& text)
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hConsole == INVALID_HANDLE_VALUE)
        return;

    DWORD written = 0;
    WriteConsoleW(hConsole, text.c_str(), static_cast<DWORD>(text.size()), &written, nullptr);
}

void PrintConsoleLine(const std::wstring& text)
{
    PrintConsoleText(text + L"\n");
}

float Clamp01(float value)
{
    return (std::max)(0.0f, (std::min)(1.0f, value));
}

float RandomFloat(float minValue, float maxValue)
{
    float t = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    return minValue + (maxValue - minValue) * t;
}

PxVec3 GetCameraForward(const PxTransform& cameraTransform)
{
    PxVec3 forward = cameraTransform.q.rotate(PxVec3(0.0f, 0.0f, -1.0f));
    return forward.getNormalized();
}

PxVec3 GetHorizontalDirection(const PxVec3& value, const PxVec3& fallback = PxVec3(1.0f, 0.0f, 0.0f))
{
    PxVec3 result(value.x, 0.0f, value.z);
    if (result.magnitudeSquared() < 0.0001f)
        return fallback;

    return result.getNormalized();
}

PxVec3 GetSpreadDirection(const PxVec3& forward)
{
    const float maxAngle = GameParams::SHOT_SPREAD_DEGREES * PxPi / 180.0f;
    const float yaw = RandomFloat(-maxAngle, maxAngle);
    const float pitch = RandomFloat(-maxAngle, maxAngle);

    PxVec3 right = forward.cross(PxVec3(0.0f, 1.0f, 0.0f));
    if (right.magnitudeSquared() < 0.0001f)
        right = PxVec3(1.0f, 0.0f, 0.0f);
    right.normalize();

    PxVec3 up = right.cross(forward).getNormalized();
    PxVec3 dir = forward + right * std::tan(yaw) + up * std::tan(pitch);
    return dir.getNormalized();
}

void RemoveActorSafe(PxRigidActor*& actor)
{
    if (!actor)
        return;

    engine->RemoveActor(actor);
    SAFE_RELEASE(actor);
}

bool IsRagdollActor(const PxRigidActor* actor)
{
    return std::find(ragdollBodies.begin(), ragdollBodies.end(), actor) != ragdollBodies.end();
}

PxRigidDynamic* CreateDynamicPart(PxShape* shape, const PxVec3& pos, float density)
{
    PxRigidDynamic* actor = engine->AddDynamicActor(shape, pos, PxQuat(PxIdentity), density);
    actor->setLinearDamping(0.10f);
    actor->setAngularDamping(0.45f);
    actor->setSolverIterationCounts(12, 4);
    ragdollBodies.push_back(actor);
    SAFE_RELEASE(shape);
    return actor;
}

PxSphericalJoint* AddLimitedBallJoint(
    PxRigidActor* parent,
    const PxVec3& parentAnchor,
    PxRigidActor* child,
    const PxVec3& childAnchor,
    float swingY,
    float swingZ)
{
    PxSphericalJoint* joint = PxSphericalJointCreate(
        *engine->GetPhysics(),
        parent,
        PxTransform(parentAnchor),
        child,
        PxTransform(childAnchor)
    );

    joint->setLimitCone(PxJointLimitCone(swingY, swingZ));
    joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
    ragdollJoints.push_back(joint);
    return joint;
}

void ActivateRagdoll(const PxVec3& impulseDir, float impulseValue)
{
    if (enemyRagdollActive || !enemyCapsule)
        return;

    PxVec3 basePos = enemyCapsule->getGlobalPose().p;
    PxVec3 inheritedVelocity = enemyCapsule->getLinearVelocity();

    PxRigidActor* oldEnemy = enemyCapsule;
    RemoveActorSafe(oldEnemy);
    enemyCapsule = nullptr;

    PxPhysics* physics = engine->GetPhysics();

    ragdollTorso = CreateDynamicPart(
        engine->CreateBoxShape(PxVec3(0.65f, 1.00f, 0.35f), enemyMaterial),
        basePos + PxVec3(0.0f, 0.45f, 0.0f),
        45.0f
    );

    ragdollHead = CreateDynamicPart(
        engine->CreateSphereShape(0.24f, enemyMaterial),
        basePos + PxVec3(0.0f, 1.20f, 0.0f),
        18.0f
    );

    PxShape* leftArmShape = engine->CreateCapsuleShape(0.11f, 0.75f, enemyMaterial);
    leftArmShape->setLocalPose(PxTransform(PxVec3(0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
    ragdollLeftArm = CreateDynamicPart(leftArmShape, basePos + PxVec3(-0.62f, 0.65f, 0.0f), 16.0f);

    PxShape* rightArmShape = engine->CreateCapsuleShape(0.11f, 0.75f, enemyMaterial);
    rightArmShape->setLocalPose(PxTransform(PxVec3(0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
    ragdollRightArm = CreateDynamicPart(rightArmShape, basePos + PxVec3(0.62f, 0.65f, 0.0f), 16.0f);

    PxShape* leftLegShape = engine->CreateCapsuleShape(0.13f, 0.95f, enemyMaterial);
    leftLegShape->setLocalPose(PxTransform(PxVec3(0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
    ragdollLeftLeg = CreateDynamicPart(leftLegShape, basePos + PxVec3(-0.22f, -0.35f, 0.0f), 22.0f);

    PxShape* rightLegShape = engine->CreateCapsuleShape(0.13f, 0.95f, enemyMaterial);
    rightLegShape->setLocalPose(PxTransform(PxVec3(0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
    ragdollRightLeg = CreateDynamicPart(rightLegShape, basePos + PxVec3(0.22f, -0.35f, 0.0f), 22.0f);

    for (PxRigidDynamic* body : ragdollBodies)
    {
        body->setLinearVelocity(inheritedVelocity);
        body->wakeUp();
    }

    // Spherical joint подходит дл€ плеч, шеи и таза, потому что разрешает вращение как шарнирное соединение,
    // а cone-limit ограничивает неестественные повороты, например голову на 360 градусов.
    AddLimitedBallJoint(ragdollTorso, PxVec3(0.0f, 0.55f, 0.0f), ragdollHead, PxVec3(0.0f, -0.24f, 0.0f), PxPi / 7.0f, PxPi / 7.0f);
    AddLimitedBallJoint(ragdollTorso, PxVec3(-0.36f, 0.32f, 0.0f), ragdollLeftArm, PxVec3(0.38f, 0.0f, 0.0f), PxPi / 2.8f, PxPi / 2.8f);
    AddLimitedBallJoint(ragdollTorso, PxVec3(0.36f, 0.32f, 0.0f), ragdollRightArm, PxVec3(-0.38f, 0.0f, 0.0f), PxPi / 2.8f, PxPi / 2.8f);
    AddLimitedBallJoint(ragdollTorso, PxVec3(-0.20f, -0.52f, 0.0f), ragdollLeftLeg, PxVec3(0.0f, 0.47f, 0.0f), PxPi / 4.0f, PxPi / 4.0f);
    AddLimitedBallJoint(ragdollTorso, PxVec3(0.20f, -0.52f, 0.0f), ragdollRightLeg, PxVec3(0.0f, 0.47f, 0.0f), PxPi / 4.0f, PxPi / 4.0f);

    enemyRagdollActive = true;

    if (ragdollTorso)
    {
        ragdollTorso->addForce(impulseDir * impulseValue, PxForceMode::eIMPULSE);
        ragdollTorso->addTorque(PxVec3(-impulseDir.z, 0.0f, impulseDir.x) * impulseValue * 0.25f, PxForceMode::eIMPULSE);
    }

    PrintConsoleLine(L"Ragdoll: противник упал.");
}

void ApplyImpulseToEnemy(const PxVec3& dir, float impulseValue)
{
    if (!enemyRagdollActive)
    {
        ActivateRagdoll(dir, impulseValue);
        return;
    }

    if (ragdollTorso)
    {
        ragdollTorso->wakeUp();
        ragdollTorso->addForce(dir * impulseValue, PxForceMode::eIMPULSE);
    }
}

void CreateEnemy(const PxVec3& pos)
{
    PxShape* enemyShape = engine->CreateCapsuleShape(
        GameParams::ENEMY_RADIUS,
        GameParams::ENEMY_HEIGHT,
        enemyMaterial
    );

    // ¬ PhysX капсула по умолчанию лежит вдоль X. ѕоворот на 90 градусов вокруг Z делает ось вертикальной Y.
    enemyShape->setLocalPose(PxTransform(PxVec3(0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));

    enemyCapsule = engine->AddDynamicActor(enemyShape, pos, PxQuat(PxIdentity), GameParams::ENEMY_DENSITY);
    enemyCapsule->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, true);
    enemyCapsule->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, true);
    enemyCapsule->setAngularDamping(8.0f);
    enemyCapsule->setMaxLinearVelocity(6.0f);
    enemyCapsule->setSolverIterationCounts(12, 4);
    enemyCapsule->setLinearDamping(0.45f);

    SAFE_RELEASE(enemyShape);
}

PxRigidStatic* AddStaticBox(const PxVec3& pos, const PxVec3& size, PxMaterial* material)
{
    PxShape* shape = engine->CreateBoxShape(size, material);
    PxRigidStatic* actor = engine->AddStaticActor(shape, pos, PxQuat(PxIdentity));
    SAFE_RELEASE(shape);

    BoxObstacle obstacle;
    obstacle.actor = actor;
    obstacle.center = pos;
    obstacle.size = size;
    obstacles.push_back(obstacle);

    return actor;
}

bool RayHitsWallBeforeTarget(const PxVec3& from, const PxVec3& to, const PxRigidActor* targetActor)
{
    PxVec3 dir = to - from;
    float distance = dir.magnitude();
    if (distance < 0.001f)
        return false;

    dir /= distance;
    PxRaycastBuffer hit;
    bool hasHit = engine->GetScene()->raycast(from, dir, distance, hit);

    if (!hasHit)
        return false;

    return hit.block.actor != targetActor;
}

bool IsEnemyProtectedFromExplosion(const PxVec3& explosionPos, const PxVec3& enemyPos)
{
    PxRigidActor* visibleEnemyActor = nullptr;

    if (!enemyRagdollActive)
        visibleEnemyActor = enemyCapsule;
    else
        visibleEnemyActor = ragdollTorso;

    return RayHitsWallBeforeTarget(
        explosionPos + PxVec3(0.0f, 0.15f, 0.0f),
        enemyPos + PxVec3(0.0f, 0.15f, 0.0f),
        visibleEnemyActor
    );
}

void Shoot(const PxTransform& cameraTransform)
{
    playerPosition = cameraTransform.p;

    PxVec3 origin = cameraTransform.p + GetCameraForward(cameraTransform) * 0.7f;
    PxVec3 dir = GetSpreadDirection(GetCameraForward(cameraTransform));

    PxRaycastBuffer hit;
    bool hasHit = engine->GetScene()->raycast(origin, dir, GameParams::SHOT_MAX_DISTANCE, hit);

    shotLineStart = origin;
    shotLineEnd = hasHit ? hit.block.position : origin + dir * GameParams::SHOT_MAX_DISTANCE;
    shotLineVisible = true;
    shotLineTimeLeft = GameParams::SHOT_VISUAL_TIME;

    if (!hasHit)
    {
        PrintConsoleLine(L"¬ыстрел: промах.");
        return;
    }

    bool enemyHit = false;
    if (!enemyRagdollActive)
        enemyHit = hit.block.actor == enemyCapsule;
    else
        enemyHit = IsRagdollActor(hit.block.actor);

    if (enemyHit)
    {
        PrintConsoleLine(L"¬ыстрел: попадание по противнику.");
        ApplyImpulseToEnemy(dir, GameParams::SHOT_IMPULSE);
    }
    else
    {
        PrintConsoleLine(L"¬ыстрел: попадание в преп€тствие.");
    }
}

void ThrowGrenade(const PxTransform& cameraTransform)
{
    playerPosition = cameraTransform.p;

    PxVec3 forward = GetCameraForward(cameraTransform);
    PxVec3 spawnPos = cameraTransform.p + forward * 1.25f + PxVec3(0.0f, -0.25f, 0.0f);

    PxShape* shape = engine->CreateSphereShape(GameParams::GRENADE_RADIUS, grenadeMaterial);
    PxRigidDynamic* grenadeActor = engine->AddDynamicActor(shape, spawnPos, PxQuat(PxIdentity), GameParams::GRENADE_DENSITY);
    SAFE_RELEASE(shape);

    grenadeActor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    grenadeActor->setLinearDamping(0.04f);
    grenadeActor->setAngularDamping(0.15f);
    grenadeActor->setLinearVelocity(forward * GameParams::GRENADE_THROW_SPEED + PxVec3(0.0f, GameParams::GRENADE_UP_BOOST, 0.0f));
    grenadeActor->wakeUp();

    grenades.push_back({ grenadeActor, GameParams::GRENADE_FUSE_TIME });
    PrintConsoleLine(L"√раната: бросок.");
}

PxVec3 GetEnemyDamageCenter()
{
    if (enemyRagdollActive && ragdollTorso)
        return ragdollTorso->getGlobalPose().p;

    if (enemyCapsule)
        return enemyCapsule->getGlobalPose().p + PxVec3(0.0f, 0.75f, 0.0f);

    return PxVec3(0.0f);
}

void ExplodeGrenade(int index)
{
    if (index < 0 || index >= static_cast<int>(grenades.size()))
        return;

    PxRigidDynamic* grenadeActor = grenades[index].actor;
    if (!grenadeActor)
        return;

    PxVec3 boomPos = grenadeActor->getGlobalPose().p;

    PxRigidActor* actorToRemove = grenadeActor;
    RemoveActorSafe(actorToRemove);
    grenades.erase(grenades.begin() + index);

    explosionCenter = boomPos;
    explosionVisible = true;
    explosionTimeLeft = GameParams::EXPLOSION_VISUAL_TIME;

    PrintConsoleLine(L"√раната: взрыв.");

    if (!enemyCapsule && !enemyRagdollActive)
        return;

    PxVec3 enemyCenter = GetEnemyDamageCenter();
    PxVec3 delta = enemyCenter - boomPos;
    float distance = delta.magnitude();

    if (distance > GameParams::GRENADE_EXPLOSION_RADIUS)
    {
        PrintConsoleLine(L"¬зрыв: противник вне радиуса.");
        return;
    }

    if (IsEnemyProtectedFromExplosion(boomPos, enemyCenter))
    {
        PrintConsoleLine(L"¬зрыв: стена закрыла противника, урон не засчитан.");
        return;
    }

    if (distance < 0.001f)
    {
        delta = PxVec3(1.0f, 0.0f, 0.0f);
        distance = 0.001f;
    }

    float factor = Clamp01(1.0f - distance / GameParams::GRENADE_EXPLOSION_RADIUS);
    float damage = GameParams::GRENADE_MAX_DAMAGE * factor;
    float impulse = GameParams::GRENADE_MAX_IMPULSE * factor;

    ApplyImpulseToEnemy(delta.getNormalized(), impulse);

    std::wcout << L"¬зрыв: противник получил урон = "
        << damage
        << L", рассто€ние = "
        << distance
        << L"\n";
}

void UpdateGrenades(float dt)
{
    for (int i = static_cast<int>(grenades.size()) - 1; i >= 0; --i)
    {
        grenades[i].fuseLeft -= dt;
        if (grenades[i].fuseLeft <= 0.0f)
            ExplodeGrenade(i);
    }
}

bool FindNearestCoverPoint(const PxVec3& enemyPos, const PxVec3& playerPos, PxVec3& outCoverPoint)
{
    bool found = false;
    float bestDistanceSq = PX_MAX_F32;

    for (const BoxObstacle& obstacle : obstacles)
    {
        PxVec3 fromPlayerToWall = GetHorizontalDirection(obstacle.center - playerPos);
        float maxHalf = (std::max)(obstacle.size.x, obstacle.size.z) * 0.5f;
        PxVec3 candidate = obstacle.center + fromPlayerToWall * (maxHalf + GameParams::COVER_OFFSET);
        candidate.y = enemyPos.y;

        float enemyToCandidateSq = (candidate - enemyPos).magnitudeSquared();
        if (enemyToCandidateSq > GameParams::ENEMY_VIEW_RADIUS * GameParams::ENEMY_VIEW_RADIUS)
            continue;

        PxVec3 playerEye = playerPos + PxVec3(0.0f, 0.8f, 0.0f);
        PxVec3 candidateEye = candidate + PxVec3(0.0f, 0.8f, 0.0f);
        PxVec3 rayDir = candidateEye - playerEye;
        float rayLength = rayDir.magnitude();

        if (rayLength < 0.001f)
            continue;

        rayDir /= rayLength;
        PxRaycastBuffer hit;
        bool blocked = engine->GetScene()->raycast(playerEye, rayDir, rayLength, hit);

        if (!blocked || hit.block.actor != obstacle.actor)
            continue;

        if (enemyToCandidateSq < bestDistanceSq)
        {
            bestDistanceSq = enemyToCandidateSq;
            outCoverPoint = candidate;
            found = true;
        }
    }

    return found;
}

void UpdateEnemyAI(float dt)
{
    if (enemyRagdollActive || !enemyCapsule)
        return;

    PxVec3 enemyPos = enemyCapsule->getGlobalPose().p;
    PxVec3 toPlayer = playerPosition - enemyPos;
    toPlayer.y = 0.0f;

    float distanceToPlayer = toPlayer.magnitude();
    if (distanceToPlayer > GameParams::ENEMY_VIEW_RADIUS)
    {
        if (enemyHasSeenPlayer)
            PrintConsoleLine(L"AI: игрок вышел из зоны обзора.");

        enemyHasSeenPlayer = false;
        enemyMovingToCover = false;

        PxVec3 oldVel = enemyCapsule->getLinearVelocity();
        enemyCapsule->setLinearVelocity(PxVec3(0.0f, oldVel.y, 0.0f));
        return;
    }

    if (!enemyHasSeenPlayer)
        PrintConsoleLine(L"AI: игрок замечен в сфере обзора.");

    enemyHasSeenPlayer = true;

    PxVec3 target(0.0f);
    bool coverFound = FindNearestCoverPoint(enemyPos, playerPosition, target);

    if (coverFound)
    {
        if (!enemyMovingToCover)
            PrintConsoleLine(L"AI: найдено укрытие, противник перемещаетс€ за стену.");
        enemyMovingToCover = true;
    }
    else
    {
        if (enemyMovingToCover)
            PrintConsoleLine(L"AI: укрытие потер€но.");

        enemyMovingToCover = false;
        PxVec3 fleeDir = GetHorizontalDirection(enemyPos - playerPosition);
        target = enemyPos + fleeDir * GameParams::FLEE_STEP;
    }

    PxVec3 moveDir = target - enemyPos;
    moveDir.y = 0.0f;

    if (moveDir.magnitudeSquared() < 0.12f)
    {
        PxVec3 oldVel = enemyCapsule->getLinearVelocity();
        enemyCapsule->setLinearVelocity(PxVec3(0.0f, oldVel.y, 0.0f));
        return;
    }

    moveDir.normalize();
    PxVec3 oldVel = enemyCapsule->getLinearVelocity();
    enemyCapsule->wakeUp();
    enemyCapsule->setLinearVelocity(PxVec3(
        moveDir.x * GameParams::ENEMY_MOVE_SPEED,
        oldVel.y,
        moveDir.z * GameParams::ENEMY_MOVE_SPEED
    ));
}

void DrawCrosshair()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    glOrtho(0, w, 0, h, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    float cx = w * 0.5f;
    float cy = h * 0.5f;
    float size = 11.0f;
    float gap = 4.0f;

    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex2f(cx - size, cy); glVertex2f(cx - gap, cy);
    glVertex2f(cx + gap, cy);  glVertex2f(cx + size, cy);
    glVertex2f(cx, cy - size); glVertex2f(cx, cy - gap);
    glVertex2f(cx, cy + gap);  glVertex2f(cx, cy + size);
    glEnd();
    glLineWidth(1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void DrawShotLine()
{
    if (!shotLineVisible)
        return;

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1.0f, 0.1f, 0.1f);
    glLineWidth(3.0f);

    glBegin(GL_LINES);
    glVertex3f(shotLineStart.x, shotLineStart.y, shotLineStart.z);
    glVertex3f(shotLineEnd.x, shotLineEnd.y, shotLineEnd.z);
    glEnd();

    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

void DrawExplosionSphere()
{
    if (!explosionVisible)
        return;

    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.9f, 0.0f);
    glLineWidth(2.0f);

    glPushMatrix();
    glTranslatef(explosionCenter.x, explosionCenter.y, explosionCenter.z);
    glutWireSphere(GameParams::GRENADE_EXPLOSION_RADIUS, 24, 24);
    glPopMatrix();

    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void BuildScene()
{
    groundMaterial = engine->GetMaterial(0.75f, 0.65f, 0.15f);
    wallMaterial = engine->GetMaterial(0.65f, 0.55f, 0.05f);
    enemyMaterial = engine->GetMaterial(0.45f, 0.35f, 0.10f);
    grenadeMaterial = engine->GetMaterial(0.35f, 0.30f, 0.80f);

    engine->AddGround(PxVec3(0.0f, 1.0f, 0.0f), 0.0f, groundMaterial);

    AddStaticBox(PxVec3(-5.0f, 1.0f, 6.0f), PxVec3(2.2f, 2.0f, 1.6f), wallMaterial);
    AddStaticBox(PxVec3(4.5f, 1.25f, 4.0f), PxVec3(1.7f, 2.5f, 3.0f), wallMaterial);
    AddStaticBox(PxVec3(-7.0f, 1.0f, -1.5f), PxVec3(2.0f, 2.0f, 2.0f), wallMaterial);
    AddStaticBox(PxVec3(6.5f, 1.0f, -4.5f), PxVec3(2.0f, 2.0f, 2.4f), wallMaterial);
    AddStaticBox(PxVec3(0.0f, 1.5f, -6.0f), PxVec3(4.0f, 3.0f, 0.8f), wallMaterial);

    CreateEnemy(PxVec3(0.0f, 1.7f, 3.5f));
}

void PrintHelp()
{
    PrintConsoleLine(L"”правление:");
    PrintConsoleLine(L"E - выстрел raycast с небольшим разбросом");
    PrintConsoleLine(L"R - бросить физическую гранату");
    PrintConsoleLine(L"WASD + мышь - стандартное управление камерой snippets");
    PrintConsoleLine(L"");
}

void keyPressedCallback(unsigned char key, const PxTransform& cameraTransform)
{
    playerPosition = cameraTransform.p;

    switch (std::toupper(key))
    {
    case 'E':
        Shoot(cameraTransform);
        break;

    case 'R':
        ThrowGrenade(cameraTransform);
        break;

    default:
        break;
    }
}

void renderCallback()
{
    const float dt = 1.0f / 60.0f;

    if (mainCamera)
        playerPosition = mainCamera->getTransform().p;

    UpdateEnemyAI(dt);
    engine->Simulate(dt);
    UpdateGrenades(dt);

    if (shotLineVisible)
    {
        shotLineTimeLeft -= dt;
        if (shotLineTimeLeft <= 0.0f)
            shotLineVisible = false;
    }

    if (explosionVisible)
    {
        explosionTimeLeft -= dt;
        if (explosionTimeLeft <= 0.0f)
            explosionVisible = false;
    }

    Snippets::startRender(mainCamera);

    std::vector<PxRigidActor*> actors = engine->GetActors();
    if (!actors.empty())
        Snippets::renderActors(actors.data(), static_cast<PxU32>(actors.size()), true);

    DrawShotLine();
    DrawExplosionSphere();
    DrawCrosshair();

    Snippets::finishRender();
}

void exitCallback()
{
    for (PxJoint* joint : ragdollJoints)
        SAFE_RELEASE(joint);
    ragdollJoints.clear();
    ragdollBodies.clear();
    grenades.clear();
    obstacles.clear();

    delete mainCamera;
    mainCamera = nullptr;

    delete engine;
    engine = nullptr;
}

int main()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::setlocale(LC_ALL, "Russian");

    mainCamera = new Snippets::Camera(
        PxVec3(0.0f, 3.2f, 14.0f),
        PxVec3(0.0f, -0.16f, -1.0f)
    );

    Snippets::setupDefault(
        "Lab 2 - Shooter Simulation",
        mainCamera,
        keyPressedCallback,
        renderCallback,
        exitCallback
    );

    engine = new PhysicsEngine();
    BuildScene();

    PrintConsoleLine(L"Ћабораторна€ работа 2 запущена.");
    PrintHelp();

    glutMainLoop();
    return 0;
}
