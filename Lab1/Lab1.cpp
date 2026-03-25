#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
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
    constexpr float TABLE_LENGTH = 2.40f;
    constexpr float TABLE_WIDTH = 1.20f;
    constexpr float TABLE_THICKNESS = 0.06f;

    constexpr float BOARD_HEIGHT = 0.10f;
    constexpr float BOARD_THICKNESS = 0.08f;

    constexpr float BALL_RADIUS = 0.0285f;
    constexpr float BALL_DENSITY = 850.0f;

    constexpr float POCKET_RADIUS = 0.055f;
    constexpr float POCKET_REMOVE_Y = -0.05f;

    constexpr float CUE_RADIUS = 0.010f;
    constexpr float CUE_LENGTH = 0.55f;
    constexpr float CUE_DENSITY = 3.0f;

    constexpr float ROTATE_STEP = 0.08f;
    constexpr float POWER_STEP = 0.35f;
    constexpr float MIN_POWER = 1.2f;
    constexpr float MAX_POWER = 6.0f;

    constexpr float STOP_LINEAR = 0.05f;
    constexpr float STOP_ANGULAR = 0.05f;

    constexpr float CUE_IDLE_OFFSET = 0.09f;
    constexpr float CUE_BACK_SWING = 0.16f;
    constexpr float CUE_FORWARD_SWING = 0.03f;
}

PhysicsEngine* engine = nullptr;
Snippets::Camera* mainCamera = nullptr;

PxRigidDynamic* cueBallActor = nullptr;
PxRigidDynamic* cueActor = nullptr;
PxRigidDynamic* blackBallActor = nullptr;

std::vector<PxRigidDynamic*> rackBalls;

PxMaterial* redBallMaterial = nullptr;
PxMaterial* whiteBallMaterial = nullptr;
PxMaterial* blackBallMaterial = nullptr;

float aimYaw = 0.0f;
float currentPower = 3.0f;

bool isStrikeAnimating = false;
bool hasAppliedStrikeImpulse = false;
float strikeAnimState = 0.0f;

bool isGameFinished = false;

PxVec3 CalculateShotDirection()
{
    return PxVec3(-sinf(aimYaw), 0.0f, -cosf(aimYaw)).getNormalized();
}

bool ExistsInScene(PxActor* actor)
{
    if (!actor)
        return false;

    std::vector<PxRigidActor*> actors = engine->GetActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    for (PxRigidActor* current : actors)
    {
        if (current == actor)
            return true;
    }
    return false;
}

void RemoveActorWithShapes(PxRigidActor*& actor)
{
    if (!actor)
        return;

    engine->RemoveActor(actor);
    SAFE_RELEASE(actor);
}

bool BallsAreMoving()
{
    std::vector<PxRigidActor*> actors = engine->GetActors(PxActorTypeFlag::eRIGID_DYNAMIC);

    for (PxRigidActor* actor : actors)
    {
        if (actor == cueActor)
            continue;

        PxRigidDynamic* body = static_cast<PxRigidDynamic*>(actor);
        if (!body)
            continue;

        if (body->getLinearVelocity().magnitude() > GameParams::STOP_LINEAR)
            return true;

        if (body->getAngularVelocity().magnitude() > GameParams::STOP_ANGULAR)
            return true;
    }

    return false;
}

void UpdateCuePlacement(float extraOffset = 0.0f)
{
    if (!cueActor || !cueBallActor || isGameFinished)
        return;

    if (!ExistsInScene(cueBallActor))
        return;

    PxVec3 whiteBallPos = cueBallActor->getGlobalPose().p;
    PxVec3 shotDir = CalculateShotDirection();

    float distance =
        GameParams::BALL_RADIUS +
        GameParams::CUE_LENGTH * 0.5f +
        GameParams::CUE_IDLE_OFFSET +
        extraOffset;

    PxVec3 cuePos = whiteBallPos - shotDir * distance;
    cuePos.y = GameParams::BALL_RADIUS;

    PxQuat cueRot =
        PxQuat(aimYaw, PxVec3(0.0f, 1.0f, 0.0f)) *
        PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f));

    cueActor->setKinematicTarget(PxTransform(cuePos, cueRot));
}

void BeginStrike()
{
    if (isGameFinished || isStrikeAnimating || BallsAreMoving())
        return;

    isStrikeAnimating = true;
    hasAppliedStrikeImpulse = false;
    strikeAnimState = 0.0f;
}

void AnimateStrike()
{
    if (!isStrikeAnimating || !cueBallActor || !ExistsInScene(cueBallActor))
        return;

    strikeAnimState += 0.08f;

    if (strikeAnimState < 0.35f)
    {
        float t = strikeAnimState / 0.35f;
        UpdateCuePlacement(GameParams::CUE_BACK_SWING * t);
    }
    else if (strikeAnimState < 0.70f)
    {
        float t = (strikeAnimState - 0.35f) / 0.35f;
        float offset =
            GameParams::CUE_BACK_SWING * (1.0f - t) +
            GameParams::CUE_FORWARD_SWING * t;

        UpdateCuePlacement(offset);

        if (!hasAppliedStrikeImpulse && t > 0.70f)
        {
            cueBallActor->addForce(CalculateShotDirection() * currentPower, PxForceMode::eIMPULSE);
            hasAppliedStrikeImpulse = true;
        }
    }
    else if (strikeAnimState < 1.0f)
    {
        float t = (strikeAnimState - 0.70f) / 0.30f;
        UpdateCuePlacement(GameParams::CUE_FORWARD_SWING * (1.0f - t));
    }
    else
    {
        isStrikeAnimating = false;
        hasAppliedStrikeImpulse = false;
        strikeAnimState = 0.0f;
        UpdateCuePlacement();
    }
}

PxRigidDynamic* CreateBall(const PxVec3& pos, PxMaterial* material)
{
    PxShape* ballShape = engine->CreateSphereShape(
        GameParams::BALL_RADIUS,
        material
    );

    PxRigidDynamic* ball = engine->AddDynamicActor(
        ballShape,
        pos,
        PxQuat(PxIdentity),
        GameParams::BALL_DENSITY
    );

    ball->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    ball->setSolverIterationCounts(12, 4);

    ball->setLinearDamping(0.18f);
    ball->setAngularDamping(0.20f);
    ball->setMaxLinearVelocity(10.0f);
    ball->setMaxAngularVelocity(25.0f);

    SAFE_RELEASE(ballShape);
    return ball;
}

void CreateTableBase(PxMaterial* material)
{
    PxShape* tableShape = engine->CreateBoxShape(
        PxVec3(GameParams::TABLE_LENGTH, GameParams::TABLE_THICKNESS, GameParams::TABLE_WIDTH),
        material
    );

    engine->AddStaticActor(
        tableShape,
        PxVec3(0.0f, -GameParams::TABLE_THICKNESS * 0.5f, 0.0f),
        PxQuat(PxIdentity)
    );

    SAFE_RELEASE(tableShape);
}

void AddBoard(const PxVec3& size, const PxVec3& pos, PxMaterial* material)
{
    PxShape* boardShape = engine->CreateBoxShape(size, material);
    engine->AddStaticActor(boardShape, pos, PxQuat(PxIdentity));
    SAFE_RELEASE(boardShape);
}

void CreateBoards(PxMaterial* material)
{
    const float L = GameParams::TABLE_LENGTH;
    const float W = GameParams::TABLE_WIDTH;
    const float H = GameParams::BOARD_HEIGHT;
    const float T = GameParams::BOARD_THICKNESS;

    const float cornerGap = GameParams::POCKET_RADIUS * 1.8f;
    const float middleGap = GameParams::POCKET_RADIUS * 1.4f;

    float longPart = (L - 2.0f * cornerGap - middleGap) * 0.5f;
    float sidePart = W - 2.0f * cornerGap;

    AddBoard(
        PxVec3(longPart, H, T),
        PxVec3(-(middleGap * 0.5f + longPart * 0.5f), H * 0.5f, -(W * 0.5f + T * 0.5f)),
        material
    );
    AddBoard(
        PxVec3(longPart, H, T),
        PxVec3((middleGap * 0.5f + longPart * 0.5f), H * 0.5f, -(W * 0.5f + T * 0.5f)),
        material
    );

    AddBoard(
        PxVec3(longPart, H, T),
        PxVec3(-(middleGap * 0.5f + longPart * 0.5f), H * 0.5f, (W * 0.5f + T * 0.5f)),
        material
    );
    AddBoard(
        PxVec3(longPart, H, T),
        PxVec3((middleGap * 0.5f + longPart * 0.5f), H * 0.5f, (W * 0.5f + T * 0.5f)),
        material
    );

    AddBoard(
        PxVec3(T, H, sidePart),
        PxVec3(-(L * 0.5f + T * 0.5f), H * 0.5f, 0.0f),
        material
    );

    AddBoard(
        PxVec3(T, H, sidePart),
        PxVec3((L * 0.5f + T * 0.5f), H * 0.5f, 0.0f),
        material
    );
}

void CreateBallPyramid()
{
    rackBalls.clear();
    blackBallActor = nullptr;

    float startX = GameParams::TABLE_LENGTH * 0.24f;
    float step = GameParams::BALL_RADIUS * 2.08f;
    float rowAdvance = step * 0.866f;

    for (int row = 0; row < 5; ++row)
    {
        for (int col = 0; col <= row; ++col)
        {
            float x = startX + rowAdvance * row;
            float z = (col - row * 0.5f) * step;

            PxMaterial* materialToUse = redBallMaterial;

            bool isBlackBall = (row == 2 && col == 1);
            if (isBlackBall)
                materialToUse = blackBallMaterial;

            PxRigidDynamic* createdBall = CreateBall(
                PxVec3(x, GameParams::BALL_RADIUS, z),
                materialToUse
            );

            if (isBlackBall)
                blackBallActor = createdBall;

            rackBalls.push_back(createdBall);
        }
    }

    float whiteX = -GameParams::TABLE_LENGTH * 0.30f;
    cueBallActor = CreateBall(
        PxVec3(whiteX, GameParams::BALL_RADIUS, 0.0f),
        whiteBallMaterial
    );
}

void CreateCue(PxMaterial* material)
{
    PxShape* cueShape = engine->CreateCapsuleShape(
        GameParams::CUE_RADIUS,
        GameParams::CUE_LENGTH,
        material,
        false,
        physx::PxShapeFlag::eVISUALIZATION
    );

    cueActor = engine->AddDynamicActor(
        cueShape,
        PxVec3(0.0f, GameParams::BALL_RADIUS, 0.0f),
        PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)),
        GameParams::CUE_DENSITY
    );

    cueActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

    SAFE_RELEASE(cueShape);
    UpdateCuePlacement();
}

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

bool IsInPocket(PxRigidDynamic* ball)
{
    if (!ball)
        return false;

    PxVec3 pos = ball->getGlobalPose().p;

    const float halfL = GameParams::TABLE_LENGTH * 0.5f;
    const float halfW = GameParams::TABLE_WIDTH * 0.5f;

    bool insideTableArea =
        pos.x >= -halfL - 0.1f && pos.x <= halfL + 0.1f &&
        pos.z >= -halfW - 0.1f && pos.z <= halfW + 0.1f;

    if (!insideTableArea)
        return false;

    return pos.y < GameParams::POCKET_REMOVE_Y;
}

void ProcessPocketedBalls()
{
    if (cueBallActor && ExistsInScene(cueBallActor) && IsInPocket(cueBallActor))
    {
        PxRigidActor* actor = cueBallActor;
        RemoveActorWithShapes(actor);
        cueBallActor = nullptr;

        isGameFinished = true;
        PrintConsoleLine(L"Поражение: биток попал в лузу.");
        return;
    }

    for (PxRigidDynamic*& ball : rackBalls)
    {
        if (!ball)
            continue;

        if (!ExistsInScene(ball))
            continue;

        if (IsInPocket(ball))
        {
            if (ball == blackBallActor)
            {
                isGameFinished = true;
                PrintConsoleLine(L"Поражение: в лузу попал чёрный шар.");
            }

            PxRigidActor* actor = ball;
            RemoveActorWithShapes(actor);
            ball = nullptr;
        }
    }
}

int CountActiveTargetBalls()
{
    int count = 0;
    for (PxRigidDynamic* ball : rackBalls)
    {
        if (ball && ExistsInScene(ball))
            ++count;
    }
    return count;
}

void UpdateGameState()
{
    if (isGameFinished)
        return;

    if (!cueBallActor || !ExistsInScene(cueBallActor))
    {
        isGameFinished = true;
        PrintConsoleLine(L"Поражение: биток был удалён со стола.");
        return;
    }

    if (CountActiveTargetBalls() == 0)
    {
        isGameFinished = true;
        PrintConsoleLine(L"Победа: все 15 шаров забиты.");
    }
}

void BuildGameScene()
{
    PxMaterial* tableMaterial = engine->GetMaterial(0.75f, 0.65f, 0.20f);

    redBallMaterial = engine->GetMaterial(0.30f, 0.25f, 0.92f);
    whiteBallMaterial = engine->GetMaterial(0.20f, 0.20f, 0.95f);
    blackBallMaterial = engine->GetMaterial(0.40f, 0.40f, 0.85f);

    PxMaterial* cueMaterial = engine->GetMaterial(0.40f, 0.35f, 0.15f);

    CreateTableBase(tableMaterial);
    CreateBoards(tableMaterial);
    CreateBallPyramid();
    CreateCue(cueMaterial);
}

void PrintHelp()
{
    PrintConsoleLine(L"Управление:");
    PrintConsoleLine(L"J / L - повернуть кий");
    PrintConsoleLine(L"I / K - увеличить / уменьшить силу удара");
    PrintConsoleLine(L"SPACE - выполнить удар");
    PrintConsoleLine(L"");
}

void keyPressedCallback(unsigned char key, const PxTransform&)
{
    if (isGameFinished)
        return;

    switch (toupper(key))
    {
    case 'J':
        if (!isStrikeAnimating && !BallsAreMoving())
        {
            aimYaw -= GameParams::ROTATE_STEP;
            UpdateCuePlacement();
        }
        break;

    case 'L':
        if (!isStrikeAnimating && !BallsAreMoving())
        {
            aimYaw += GameParams::ROTATE_STEP;
            UpdateCuePlacement();
        }
        break;

    case 'I':
        currentPower = (std::min)(GameParams::MAX_POWER, currentPower + GameParams::POWER_STEP);
        PrintConsoleLine(L"Сила удара:");
        std::wcout << currentPower << L"\n";
        break;

    case 'K':
        currentPower = (std::max)(GameParams::MIN_POWER, currentPower - GameParams::POWER_STEP);
        PrintConsoleLine(L"Сила удара:");
        std::wcout << currentPower << L"\n";
        break;

    case ' ':
        BeginStrike();
        break;

    default:
        break;
    }
}

void renderCallback()
{
    engine->Simulate(1.0f / 60.0f);

    AnimateStrike();
    ProcessPocketedBalls();
    UpdateGameState();

    if (!isStrikeAnimating && !BallsAreMoving() && cueBallActor && ExistsInScene(cueBallActor))
        UpdateCuePlacement();

    Snippets::startRender(mainCamera);

    std::vector<PxRigidActor*> actors = engine->GetActors();
    if (!actors.empty())
        Snippets::renderActors(actors.data(), static_cast<PxU32>(actors.size()));

    Snippets::finishRender();
}

void exitCallback()
{
    delete mainCamera;
    delete engine;
}

int main()
{
    mainCamera = new Snippets::Camera(
        PxVec3(0.0f, 3.8f, 3.3f),
        PxVec3(0.0f, -0.85f, -0.7f)
    );

    Snippets::setupDefault(
        "Billiards Simulation",
        mainCamera,
        keyPressedCallback,
        renderCallback,
        exitCallback
    );

    engine = new PhysicsEngine();
    BuildGameScene();

    PrintConsoleLine(L"Симуляция бильярда запущена.");
    PrintHelp();

    glutMainLoop();
    return 0;
}