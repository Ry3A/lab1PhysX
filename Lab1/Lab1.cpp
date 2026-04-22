#include <vector>
#include <algorithm>
#include <cmath>
#include <cctype>
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
    constexpr float SHOT_IMPULSE = 1.0f;

    constexpr float STOP_LINEAR = 0.05f;
    constexpr float STOP_ANGULAR = 0.05f;

    constexpr float CUE_IDLE_OFFSET = 0.09f;
    constexpr float CUE_BACK_SWING = 0.16f;
    constexpr float CUE_FORWARD_SWING = 0.03f;

    constexpr float STRIKE_ANIM_SPEED = 0.08f;
    constexpr float STRIKE_BACK_PHASE = 0.35f;
    constexpr float STRIKE_HIT_PHASE = 0.70f;
    constexpr float STRIKE_TOTAL_PHASE = 1.0f;
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

bool isStrikeAnimating = false;
bool hasAppliedStrikeImpulse = false;
float strikeAnimState = 0.0f;

bool isGameFinished = false;

PxVec3 CalculateShotDirection()
{
    return PxVec3(-sinf(aimYaw), 0.0f, -cosf(aimYaw)).getNormalized();
}

template <typename T>
void RemoveActorSafe(T*& actor)
{
    if (!actor)
        return;

    engine->RemoveActor(actor);
    SAFE_RELEASE(actor);
}

bool IsBodyMoving(PxRigidDynamic* body)
{
    if (!body)
        return false;

    return body->getLinearVelocity().magnitude() > GameParams::STOP_LINEAR ||
        body->getAngularVelocity().magnitude() > GameParams::STOP_ANGULAR;
}

bool BallsAreMoving()
{
    if (IsBodyMoving(cueBallActor))
        return true;

    for (PxRigidDynamic* ball : rackBalls)
    {
        if (IsBodyMoving(ball))
            return true;
    }

    return false;
}

void UpdateCuePlacement(float extraOffset = 0.0f)
{
    if (!cueActor || !cueBallActor || isGameFinished)
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

void ResetStrikeAnimation()
{
    isStrikeAnimating = false;
    hasAppliedStrikeImpulse = false;
    strikeAnimState = 0.0f;
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
    if (!isStrikeAnimating)
        return;

    if (!cueBallActor)
    {
        ResetStrikeAnimation();
        return;
    }

    strikeAnimState += GameParams::STRIKE_ANIM_SPEED;

    if (strikeAnimState < GameParams::STRIKE_BACK_PHASE)
    {
        float t = strikeAnimState / GameParams::STRIKE_BACK_PHASE;
        UpdateCuePlacement(GameParams::CUE_BACK_SWING * t);
    }
    else if (strikeAnimState < GameParams::STRIKE_HIT_PHASE)
    {
        float t = (strikeAnimState - GameParams::STRIKE_BACK_PHASE) /
            (GameParams::STRIKE_HIT_PHASE - GameParams::STRIKE_BACK_PHASE);

        float offset =
            GameParams::CUE_BACK_SWING * (1.0f - t) +
            GameParams::CUE_FORWARD_SWING * t;

        UpdateCuePlacement(offset);

        if (!hasAppliedStrikeImpulse && t > 0.70f)
        {
            cueBallActor->addForce(
                CalculateShotDirection() * GameParams::SHOT_IMPULSE,
                PxForceMode::eIMPULSE
            );
            hasAppliedStrikeImpulse = true;
        }
    }
    else if (strikeAnimState < GameParams::STRIKE_TOTAL_PHASE)
    {
        float t = (strikeAnimState - GameParams::STRIKE_HIT_PHASE) /
            (GameParams::STRIKE_TOTAL_PHASE - GameParams::STRIKE_HIT_PHASE);

        UpdateCuePlacement(GameParams::CUE_FORWARD_SWING * (1.0f - t));
    }
    else
    {
        ResetStrikeAnimation();
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

            bool isBlackBall = (row == 2 && col == 1);
            PxMaterial* materialToUse = isBlackBall ? blackBallMaterial : redBallMaterial;

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

    return ball->getGlobalPose().p.y < GameParams::POCKET_REMOVE_Y;
}

void ProcessPocketedBalls()
{
    if (cueBallActor && IsInPocket(cueBallActor))
    {
        RemoveActorSafe(cueBallActor);
        isGameFinished = true;
        PrintConsoleLine(L"Поражение: биток попал в лузу.");
        return;
    }

    for (PxRigidDynamic*& ball : rackBalls)
    {
        if (!ball || !IsInPocket(ball))
            continue;

        if (ball == blackBallActor)
        {
            isGameFinished = true;
            PrintConsoleLine(L"Поражение: в лузу попал чёрный шар.");
        }

        RemoveActorSafe(ball);
    }
}

int CountActiveTargetBalls()
{
    return static_cast<int>(std::count_if(
        rackBalls.begin(),
        rackBalls.end(),
        [](PxRigidDynamic* ball)
        {
            return ball != nullptr;
        }
    ));
}

void UpdateGameState()
{
    if (isGameFinished)
        return;

    if (!cueBallActor)
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
    PrintConsoleLine(L"SPACE - выполнить удар");
    PrintConsoleLine(L"");
}

void keyPressedCallback(unsigned char key, const PxTransform&)
{
    if (isGameFinished)
        return;

    switch (std::toupper(key))
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

    if (!isStrikeAnimating && !BallsAreMoving() && cueBallActor)
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