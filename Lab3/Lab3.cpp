#define NOMINMAX

#include <vector>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <cstdint>
#include <iostream>
#include <windows.h>

#include "PhysicsEngine.h"
#include "Cloth.h"

#include "NvCloth/Solver.h"

#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

using namespace physx;

#define SAFE_DELETE(obj) { \
    if (obj) {             \
        delete obj;        \
        obj = nullptr;     \
    }                      \
}

#define SAFE_RELEASE(obj) { \
    if (obj) {              \
        obj->release();     \
        obj = nullptr;      \
    }                       \
}

namespace GameParams
{
    // Шаг симуляции. Используем 60 кадров в секунду, как и в прошлых лабораторных.
    constexpr float TIME_STEP = 1.0f / 60.0f;

    // Размер основного баннера по горизонтали.
    constexpr float BANNER_WIDTH = 5.0f;

    // Размер основного баннера по вертикали.
    constexpr float BANNER_HEIGHT = 2.2f;

    // Глубина V-образного выреза у основного баннера.
    // Благодаря этому баннер отличается от обычного прямоугольника.
    constexpr float BANNER_NOTCH_DEPTH = 1.15f;

    // Количество вершин основного баннера по горизонтали.
    constexpr int BANNER_COLUMNS = 26;

    // Количество вершин основного баннера по вертикали.
    constexpr int BANNER_ROWS = 14;

    // Размер второго прямоугольного флага по горизонтали.
    constexpr float RECT_FLAG_WIDTH = 3.2f;

    // Размер второго прямоугольного флага по вертикали.
    constexpr float RECT_FLAG_HEIGHT = 2.0f;

    // Количество вершин второго флага по горизонтали.
    constexpr int RECT_FLAG_COLUMNS = 18;

    // Количество вершин второго флага по вертикали.
    constexpr int RECT_FLAG_ROWS = 12;

    // Базовая скорость ветра.
    constexpr float WIND_BASE_SPEED = 8.0f;

    // Дополнительная переменная часть ветра.
    constexpr float WIND_GUST_SPEED = 5.2f;

    // Скорость изменения направления ветра.
    constexpr float WIND_DIRECTION_SPEED = 0.85f;

    // Высота опорного шеста основного баннера.
    constexpr float MAIN_POLE_HEIGHT = 4.4f;

    // Высота опорного шеста второго флага.
    constexpr float SIDE_POLE_HEIGHT = 3.2f;

    // Толщина визуальных опор.
    constexpr float POLE_THICKNESS = 0.08f;
}

// Данные меша ткани: вершины, индексы треугольников и обратные массы частиц.
// Обратная масса 0 означает закреплённую вершину.
struct ClothMeshData
{
    std::vector<PxVec3> points;
    std::vector<uint32_t> triangles;
    std::vector<float> invMasses;
};

// Объект ткани на сцене.
// Хранит сам NvCloth-объект, индексы для рендера и цвет.
struct ClothRenderObject
{
    Cloth* cloth = nullptr;
    std::vector<uint32_t> indices;
    PxVec3 color = PxVec3(1.0f, 1.0f, 1.0f);
};


PhysicsEngine* physicsEngine = nullptr;

// Камера snippets, через которую пользователь смотрит на сцену.
Snippets::Camera* mainCamera = nullptr;

// Solver NvCloth. Он отвечает за шаги симуляции всех тканей.
nv::cloth::Solver* clothSolver = nullptr;

// Материалы PhysX для земли и декоративных опор.
PxMaterial* groundMaterial = nullptr;
PxMaterial* poleMaterial = nullptr;

// Основной баннер неправильной формы.
ClothRenderObject mainBanner;

// Второй прямоугольный флаг для дополнительного задания.
ClothRenderObject sideFlag;

// Текущее время работы сцены. Используется для плавного изменения ветра.
float simulationTime = 0.0f;

// Текущий вектор ветра. Используется и в симуляции, и для визуальной стрелки.
PxVec3 currentWind(0.0f, 0.0f, 0.0f);

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

// Возвращает позицию частицы NvCloth как PxVec3.
// В PxVec4 компонента w хранит обратную массу, а xyz — позицию.
PxVec3 ToVec3(const PxVec4& value)
{
    return PxVec3(value.x, value.y, value.z);
}

// Используется для земли, шеста и верхних/боковых планок.
PxRigidStatic* AddStaticBox(const PxVec3& position, const PxVec3& size, PxMaterial* material)
{
    PxShape* shape = physicsEngine->CreateBoxShape(size, material);
    PxRigidStatic* actor = physicsEngine->AddStaticActor(shape, position, PxQuat(PxIdentity));
    SAFE_RELEASE(shape);

    return actor;
}

// Добавляет два треугольника для одной клетки сетки ткани.
void AddGridCellTriangles(
    ClothMeshData& mesh,
    uint32_t topLeft,
    uint32_t topRight,
    uint32_t bottomLeft,
    uint32_t bottomRight)
{
    mesh.triangles.push_back(topLeft);
    mesh.triangles.push_back(bottomLeft);
    mesh.triangles.push_back(topRight);

    mesh.triangles.push_back(topRight);
    mesh.triangles.push_back(bottomLeft);
    mesh.triangles.push_back(bottomRight);
}

// Создаёт основной баннер неправильной формы.
// Верхние крайние точки закреплены, а правый край имеет V-образный вырез.
ClothMeshData CreateShapedBannerMesh(const PxVec3& topLeft)
{
    ClothMeshData mesh;

    for (int row = 0; row < GameParams::BANNER_ROWS; ++row)
    {
        float v = static_cast<float>(row) / static_cast<float>(GameParams::BANNER_ROWS - 1);

        // Вырез максимален в середине правого края и исчезает сверху/снизу.
        float middleFactor = 1.0f - std::fabs(2.0f * v - 1.0f);
        float rowWidth = GameParams::BANNER_WIDTH - GameParams::BANNER_NOTCH_DEPTH * middleFactor;

        for (int column = 0; column < GameParams::BANNER_COLUMNS; ++column)
        {
            float u = static_cast<float>(column) / static_cast<float>(GameParams::BANNER_COLUMNS - 1);

            PxVec3 point;
            point.x = topLeft.x + rowWidth * u;
            point.y = topLeft.y - GameParams::BANNER_HEIGHT * v;
            point.z = topLeft.z + 0.08f * std::sin(u * PxPi * 2.0f);

            // Закрепляем только две верхние крайние точки баннера.
            bool isPinned =
                row == 0 &&
                (column == 0 || column == GameParams::BANNER_COLUMNS - 1);

            mesh.points.push_back(point);
            mesh.invMasses.push_back(isPinned ? 0.0f : 1.0f);
        }
    }

    for (int row = 0; row < GameParams::BANNER_ROWS - 1; ++row)
    {
        for (int column = 0; column < GameParams::BANNER_COLUMNS - 1; ++column)
        {
            uint32_t topLeftIndex = row * GameParams::BANNER_COLUMNS + column;
            uint32_t topRightIndex = topLeftIndex + 1;
            uint32_t bottomLeftIndex = (row + 1) * GameParams::BANNER_COLUMNS + column;
            uint32_t bottomRightIndex = bottomLeftIndex + 1;

            AddGridCellTriangles(
                mesh,
                topLeftIndex,
                topRightIndex,
                bottomLeftIndex,
                bottomRightIndex
            );
        }
    }

    return mesh;
}

// Создаёт второй прямоугольный флаг.
// Он закреплён в двух крайних точках левого бокового края.
ClothMeshData CreateRectangularSideFlagMesh(const PxVec3& topLeft)
{
    ClothMeshData mesh;

    for (int row = 0; row < GameParams::RECT_FLAG_ROWS; ++row)
    {
        float v = static_cast<float>(row) / static_cast<float>(GameParams::RECT_FLAG_ROWS - 1);

        for (int column = 0; column < GameParams::RECT_FLAG_COLUMNS; ++column)
        {
            float u = static_cast<float>(column) / static_cast<float>(GameParams::RECT_FLAG_COLUMNS - 1);

            PxVec3 point;
            point.x = topLeft.x + GameParams::RECT_FLAG_WIDTH * u;
            point.y = topLeft.y - GameParams::RECT_FLAG_HEIGHT * v;
            point.z = topLeft.z;

            // Закрепляем две боковые крайние точки прямоугольного флага.
            // Это левый верхний и левый нижний углы, поэтому флаг держится на боковой стороне.
            bool isPinned =
                column == 0 &&
                (row == 0 || row == GameParams::RECT_FLAG_ROWS - 1);

            mesh.points.push_back(point);
            mesh.invMasses.push_back(isPinned ? 0.0f : 1.0f);
        }
    }

    for (int row = 0; row < GameParams::RECT_FLAG_ROWS - 1; ++row)
    {
        for (int column = 0; column < GameParams::RECT_FLAG_COLUMNS - 1; ++column)
        {
            uint32_t topLeftIndex = row * GameParams::RECT_FLAG_COLUMNS + column;
            uint32_t topRightIndex = topLeftIndex + 1;
            uint32_t bottomLeftIndex = (row + 1) * GameParams::RECT_FLAG_COLUMNS + column;
            uint32_t bottomRightIndex = bottomLeftIndex + 1;

            AddGridCellTriangles(
                mesh,
                topLeftIndex,
                topRightIndex,
                bottomLeftIndex,
                bottomRightIndex
            );
        }
    }

    return mesh;
}

// Настраивает параметры ткани после создания.
// Drag и lift нужны, чтобы ветер заметно влиял на ткань.
void ConfigureCloth(Cloth* cloth)
{
    if (!cloth)
        return;

    cloth->SetDamping(PxVec3(0.55f, 0.55f, 0.55f));
    cloth->SetDragCoefficient(0.18f);
    cloth->SetLiftCoefficient(0.12f);

    cloth->Get()->setSolverFrequency(240.0f);
    cloth->Get()->setTetherConstraintStiffness(1.0f);
}

// Создаёт объект ткани, добавляет его в NvCloth solver и сохраняет индексы для отрисовки.
void CreateClothObject(ClothRenderObject& object, const ClothMeshData& mesh, const PxVec3& color)
{
    object.cloth = new Cloth(mesh.points, mesh.triangles, mesh.invMasses);
    object.indices = mesh.triangles;
    object.color = color;

    ConfigureCloth(object.cloth);

    if (clothSolver && object.cloth)
        clothSolver->addCloth(object.cloth->Get());
}

// Создаёт PhysX-декорации сцены: землю, шесты и планки для флагов.
void CreateSceneGeometry()
{
    groundMaterial = physicsEngine->GetMaterial(0.75f, 0.65f, 0.10f);
    poleMaterial = physicsEngine->GetMaterial(0.50f, 0.45f, 0.20f);

    physicsEngine->AddGround(PxVec3(0.0f, 1.0f, 0.0f), 0.0f, groundMaterial);

    // Опора основного баннера.
    AddStaticBox(
        PxVec3(-5.0f, 2.2f, 0.0f),
        PxVec3(GameParams::POLE_THICKNESS, GameParams::MAIN_POLE_HEIGHT, GameParams::POLE_THICKNESS),
        poleMaterial
    );

    // Верхняя планка основного баннера.
    AddStaticBox(
        PxVec3(-5.0f + GameParams::BANNER_WIDTH * 0.5f, 4.35f, 0.0f),
        PxVec3(GameParams::BANNER_WIDTH + 0.25f, GameParams::POLE_THICKNESS, GameParams::POLE_THICKNESS),
        poleMaterial
    );

    // Опора второго прямоугольного флага.
    AddStaticBox(
        PxVec3(2.4f, 2.0f, -1.0f),
        PxVec3(GameParams::POLE_THICKNESS, GameParams::SIDE_POLE_HEIGHT, GameParams::POLE_THICKNESS),
        poleMaterial
    );

    // Боковая планка второго прямоугольного флага.
    AddStaticBox(
        PxVec3(2.4f, 2.8f, -1.0f),
        PxVec3(GameParams::POLE_THICKNESS, GameParams::RECT_FLAG_HEIGHT + 0.25f, GameParams::POLE_THICKNESS),
        poleMaterial
    );
}

// Создаёт оба флага: основной баннер неправильной формы и дополнительный прямоугольный флаг.
void CreateCloths()
{
    ClothMeshData bannerMesh = CreateShapedBannerMesh(PxVec3(-5.0f, 4.35f, 0.0f));
    ClothMeshData sideFlagMesh = CreateRectangularSideFlagMesh(PxVec3(2.4f, 3.75f, -1.0f));

    CreateClothObject(mainBanner, bannerMesh, PxVec3(0.15f, 0.35f, 0.95f));
    CreateClothObject(sideFlag, sideFlagMesh, PxVec3(0.85f, 0.78f, 0.25f));

    PrintConsoleLine(L"Создан основной баннер: 26 x 14 = 364 вершины.");
    PrintConsoleLine(L"Создан второй флаг: 18 x 12 = 216 вершин.");
}

// Обновляет ветер.
// Направление и сила меняются со временем по синусам, поэтому ткань двигается не одинаково.
void UpdateWind(float dt)
{
    simulationTime += dt;

    float gust =
        GameParams::WIND_BASE_SPEED +
        GameParams::WIND_GUST_SPEED * (0.5f + 0.5f * std::sin(simulationTime * 1.7f));

    PxVec3 direction(
        0.25f * std::sin(simulationTime * 0.8f),
        0.04f * std::sin(simulationTime * 1.1f),
        0.8f + 0.3f * std::sin(simulationTime * 0.6f)
    );

    direction.normalize();

    currentWind = direction * gust;

    if (mainBanner.cloth)
        mainBanner.cloth->SetWindVelocity(currentWind);

    if (sideFlag.cloth)
        sideFlag.cloth->SetWindVelocity(currentWind);
}

// Выполняет один шаг симуляции NvCloth.
void SimulateCloths(float dt)
{
    if (!clothSolver)
        return;

    clothSolver->beginSimulation(dt);

    for (int i = 0; i < clothSolver->getSimulationChunkCount(); ++i)
        clothSolver->simulateChunk(i);

    clothSolver->endSimulation();
}

// Рисует поверхность ткани треугольниками.
void DrawClothSurface(const ClothRenderObject& object)
{
    if (!object.cloth)
        return;

    nv::cloth::MappedRange<PxVec4> particles = object.cloth->Get()->getCurrentParticles();

    glDisable(GL_LIGHTING);
    glColor3f(object.color.x, object.color.y, object.color.z);

    glBegin(GL_TRIANGLES);

    for (size_t i = 0; i + 2 < object.indices.size(); i += 3)
    {
        PxVec3 p0 = ToVec3(particles[object.indices[i]]);
        PxVec3 p1 = ToVec3(particles[object.indices[i + 1]]);
        PxVec3 p2 = ToVec3(particles[object.indices[i + 2]]);

        PxVec3 normal = (p1 - p0).cross(p2 - p0);
        if (normal.magnitudeSquared() > 0.0001f)
        {
            normal.normalize();
            glNormal3f(normal.x, normal.y, normal.z);
        }

        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
        glVertex3f(p2.x, p2.y, p2.z);
    }

    glEnd();

    glEnable(GL_LIGHTING);
}

// Рисует сетку ткани поверх поверхности.
void DrawClothWireframe(const ClothRenderObject& object)
{
    if (!object.cloth)
        return;

    nv::cloth::MappedRange<PxVec4> particles = object.cloth->Get()->getCurrentParticles();

    glDisable(GL_LIGHTING);
    glColor3f(0.02f, 0.02f, 0.02f);
    glLineWidth(1.0f);

    glBegin(GL_LINES);

    for (size_t i = 0; i + 2 < object.indices.size(); i += 3)
    {
        PxVec3 p0 = ToVec3(particles[object.indices[i]]);
        PxVec3 p1 = ToVec3(particles[object.indices[i + 1]]);
        PxVec3 p2 = ToVec3(particles[object.indices[i + 2]]);

        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);

        glVertex3f(p1.x, p1.y, p1.z);
        glVertex3f(p2.x, p2.y, p2.z);

        glVertex3f(p2.x, p2.y, p2.z);
        glVertex3f(p0.x, p0.y, p0.z);
    }

    glEnd();
    glEnable(GL_LIGHTING);
}

// Рисует закреплённые точки ткани красными точками.
// У закреплённых частиц обратная масса равна нулю.
void DrawPinnedParticles(const ClothRenderObject& object)
{
    if (!object.cloth)
        return;

    nv::cloth::MappedRange<PxVec4> particles = object.cloth->Get()->getCurrentParticles();

    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.1f, 0.1f);
    glPointSize(8.0f);

    glBegin(GL_POINTS);

    for (uint32_t i = 0; i < object.cloth->GetNumParticles(); ++i)
    {
        if (particles[i].w == 0.0f)
        {
            PxVec3 p = ToVec3(particles[i]);
            glVertex3f(p.x, p.y, p.z);
        }
    }

    glEnd();

    glPointSize(1.0f);
    glEnable(GL_LIGHTING);
}

// Рисует один объект ткани: поверхность, сетку и закреплённые точки.
void DrawClothObject(const ClothRenderObject& object)
{
    DrawClothSurface(object);
    DrawClothWireframe(object);
    DrawPinnedParticles(object);
}

// Рисует стрелку ветра.
// Она не влияет на физику, а только визуально показывает направление currentWind.
void DrawWindArrow()
{
    PxVec3 start(-4.8f, 0.35f, 2.0f);

    PxVec3 dir = currentWind;
    if (dir.magnitudeSquared() < 0.0001f)
        return;

    dir.normalize();
    PxVec3 end = start + dir * 1.8f;

    glDisable(GL_LIGHTING);
    glColor3f(0.2f, 0.9f, 1.0f);
    glLineWidth(4.0f);

    glBegin(GL_LINES);
    glVertex3f(start.x, start.y, start.z);
    glVertex3f(end.x, end.y, end.z);
    glEnd();

    glPointSize(9.0f);
    glBegin(GL_POINTS);
    glVertex3f(end.x, end.y, end.z);
    glEnd();

    glPointSize(1.0f);
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Создаёт всю сцену лабораторной работы.
void BuildScene()
{
    clothSolver = physicsEngine->GetClothFactory()->createSolver();

    CreateSceneGeometry();
    CreateCloths();
}

// Печатает подсказку по сцене в консоль.
void PrintHelp()
{
    PrintConsoleLine(L"Лабораторная работа 3: симуляция ткани NvCloth.");
    PrintConsoleLine(L"На сцене есть основной баннер неправильной формы и второй прямоугольный флаг.");
    PrintConsoleLine(L"Красные точки показывают закреплённые вершины.");
    PrintConsoleLine(L"Голубая стрелка показывает текущее направление ветра.");
    PrintConsoleLine(L"WASD + мышь - стандартное управление камерой snippets.");
    PrintConsoleLine(L"");
}

// Обрабатывает нажатия клавиш.
void keyPressedCallback(unsigned char key, const PxTransform&)
{
    switch (std::toupper(key))
    {
    default:
        break;
    }
}

// Выполняет один кадр симуляции и отрисовки.
void renderCallback()
{
    const float dt = GameParams::TIME_STEP;

    UpdateWind(dt);
    SimulateCloths(dt);

    // PhysX нужен для отображения статических объектов сцены.
    physicsEngine->Simulate(dt);

    Snippets::startRender(mainCamera);

    std::vector<PxRigidActor*> actors = physicsEngine->GetActors();
    if (!actors.empty())
        Snippets::renderActors(actors.data(), static_cast<PxU32>(actors.size()), true);

    DrawClothObject(mainBanner);
    DrawClothObject(sideFlag);
    DrawWindArrow();

    Snippets::finishRender();
}

// Освобождает ресурсы перед выходом из приложения.
void exitCallback()
{
    if (clothSolver && mainBanner.cloth)
        clothSolver->removeCloth(mainBanner.cloth->Get());

    if (clothSolver && sideFlag.cloth)
        clothSolver->removeCloth(sideFlag.cloth->Get());

    SAFE_DELETE(mainBanner.cloth);
    SAFE_DELETE(sideFlag.cloth);

    if (clothSolver)
    {
        NV_CLOTH_DELETE(clothSolver);
        clothSolver = nullptr;
    }

    delete mainCamera;
    mainCamera = nullptr;

    delete physicsEngine;
    physicsEngine = nullptr;
}


int main()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::setlocale(LC_ALL, "Russian");

    mainCamera = new Snippets::Camera(
        PxVec3(0.0f, 3.0f, 9.0f),
        PxVec3(0.0f, -0.12f, -1.0f)
    );

    Snippets::setupDefault(
        "Lab 3 - NvCloth Flag Simulation",
        mainCamera,
        keyPressedCallback,
        renderCallback,
        exitCallback
    );

    physicsEngine = new PhysicsEngine();
    BuildScene();

    PrintHelp();

    glutMainLoop();
    return 0;
}
