#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "physics.h"
#include <random>

class Game : public olc::PixelGameEngine {
private:
    Scene scene;
    int n_iter = 4;

    static float Random(float a, float b) {
        std::random_device rd;
        static std::mt19937 m(rd());
        std::uniform_real_distribution<> dist(a, b);

        return dist(m);
    };
public:
    Game() {
        sAppName = "Physics Game";
    }

    bool OnUserCreate() override {

        scene.Initialize({ ScreenWidth() * 1.0f, ScreenHeight() * 1.0f });

        scene.AddShape({ ScreenWidth() * 0.5f, ScreenHeight() * 1.25f }, 4, ScreenWidth() * 0.5f, PI/4.0f, 0.0f, 0.0f);
        scene.AddShape({ ScreenWidth() * 0.5f, ScreenHeight() * 0.5f }, 5, 25.0f, PI/4.0f, 0.0f, 25.0f);

        Constraint rope({ ScreenWidth() * 0.25f, ScreenHeight() * 0.25f }, 60.0f, 1.5f, 0.2f, true);
        rope.Attach(scene.GetShapes()[1].GetID());
        scene.AddConstraint(rope);

        return true;
    }

    bool OnUserUpdate(float dt) override {

        // Input
        if (GetMouse(0).bPressed) {
            scene.AddShape(
                GetMousePos() * 1.0f,                                   // Position       
                3 + rand() % 3,                                         // Vertices
                10.0f + rand() % 10,                                    // Edge length
                0.0f,                                                   // Initial angle
                0.0f,                                                   // Angular velocity
                Random(2.0f, 10.0f),                                    // Mass
                olc::Pixel(rand() % 256, rand() % 256, rand() % 256),   // Color
                Random(0.2f, 0.6f),                                     // Coefficient of restitution
                0.4f,                                                   // Coefficient of static friction
                0.2f                                                    // Coefficient of dynamic friction
            );
        }

        bool is_rotate = GetKey(olc::SPACE).bHeld;
        bool is_shift = GetKey(olc::SHIFT).bHeld;

        if (is_shift && GetMouse(1).bPressed) {
            RigidBody rb(
                GetMousePos() * 1.0f,
                4,
                100.0f,
                is_rotate * PI/2.0f,
                0.0f,
                50.0f,
                0.2f,
                0.4f,
                0.2f
            );

            rb.SetColor(olc::Pixel(rand() % 256, rand() % 256, rand() % 256));

            rb.SetModel({
                { -0.5f, -0.05f },
                {  0.5f, -0.05f },
                {  0.5f,  0.05f },
                { -0.5f,  0.05f } 
            });
            scene.AddShape(rb);
        }

        if (!is_shift && GetMouse(1).bHeld) {
            for (const auto& c : scene.GetConstraints()) {
                if (scene.GetShapes()[c.GetID()].IsContainPoint(GetMousePos() * 1.0f)) {
                    scene.GetShapes()[c.GetID()].SetInputState(true);
                    scene.GetShapes()[c.GetID()].SetPosition(GetMousePos() * 1.0f);
                    //scene.GetShapes()[c.GetID()].SetVelocity({ 0.0f, 0.0f });
                }
            }
        }

        if (GetMouse(1).bReleased) {

            const auto& constraints = scene.GetConstraints();

            for (int i = 0; i < constraints.size(); i++) {
                if (scene.GetShapes()[constraints[i].GetID()].GetInputState()) {
                    scene.GetShapes()[constraints[i].GetID()].SetInputState(false);
                    if (constraints[i].IsSling()) {
                        scene.GetConstraints()[i].ApplyForces(scene.GetShapes()[constraints[i].GetID()], 1.0f, true);
                        scene.GetConstraints()[i].Reset();
                    }
                }
            }
        }

        // Logic
        for (int i = 0; i < n_iter; i++) scene.Update(dt);

        // Draw
        Clear(olc::BLACK);
        scene.Draw(this, true);

        return true;
    }
};

int main() {

    Game game;
    if (game.Construct(256, 256, 1, 1)) {
        game.Start();
    }

    return 0;
}