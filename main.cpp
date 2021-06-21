#define OLC_PGE_APPLICATION
#define OLC_PGEX_FUI
#include "olcPixelGameEngine.h"
#include "olcPGEX_FrostUI.h"
#include "physics.h"
#include <random>

float Sign(float value) { return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f); }

class RigidBodyDraw {
private:
    olc::vf2d pos, scale = { 1.0f, 1.0f };
    float len = 10.0f, angle = 0.0f;

    std::vector<olc::vf2d> model, vertices;

    olc::FrostUI gui;

    int n = 4;
    float angular_velocity = 0.0f, mass = 1.0f;
    float e = 0.2f, sf = 0.8f, df = 0.4f;

    bool is_preview = false;
public:
    int index = 0;
    void Initialize() {
        gui.create_window("gui", "Physics Settings", { 0, 0 }, { 200, 125 });
        gui.set_active_window("gui");
    
        gui.add_int_slider("vertices", "Vertex ", { 80, 5 }, { 100, 10 }, { 0, 10 }, &n);
        gui.add_float_slider("angular_velocity", "AngleVel ", { 80, 20 }, { 100, 10 }, { -5.0f, 5.0f }, &angular_velocity);
        gui.add_float_slider("mass", "Mass ", { 80, 35 }, { 100, 10 }, { 0.0f, 100.0f }, &mass);
        gui.add_float_slider("restitution", "Restitution ", { 80, 50 }, { 100, 10 }, { 0.0f, 2.0f }, &e);
        gui.add_float_slider("static_friction", "sFriction ", { 80, 65 }, { 100, 10 }, { 0.0f, 5.0f }, &sf);
        gui.add_float_slider("dynamic_friction", "dFriction ", { 80, 80 }, { 100, 10 }, { 0.0f, 5.0f }, &df);

        gui.find_element("vertices")->set_text_color(olc::Pixel(0, 0, 255));
        gui.find_element("angular_velocity")->set_text_color(olc::Pixel(50, 0, 255));
        gui.find_element("mass")->set_text_color(olc::Pixel(100, 0, 255));
        gui.find_element("restitution")->set_text_color(olc::Pixel(150, 0, 255));
        gui.find_element("static_friction")->set_text_color(olc::Pixel(200, 0, 255));
        gui.find_element("dynamic_friction")->set_text_color(olc::Pixel(255, 0, 255));
    }

    void OnMousePress() {
        model.resize(n);
        vertices.resize(n);

        for (int i = 0; i < n; i++) {
            model[i] = { cosf(2.0f * PI / n * i + 0.25f * PI), sinf(2.0f * PI / n * i + 0.25f * PI) };
        }
    }

    void OnMouseInput(const olc::vf2d& m_pos, float m_wheel, int key) {
        pos = m_pos;

        switch (key) {
        case 0: // A
            scale.x = std::fabs(std::fmin(scale.x + m_wheel, 1.0f));
            break;    
        case 1: // W
            angle += m_wheel;
            break;
        case 2: // D
            scale.y = std::fabs(std::fmin(scale.y + m_wheel, 1.0f));
            break;
        case 3: // S
            len = std::fmax(len + Sign(m_wheel), 1.0f);
            break;
        }
        
        is_preview = true;
    }

    void OnMouseRelease(Scene& scene) {
        RigidBody rb(
            pos,
            n,
            len,
            angle,
            0.0f,
            10.0f
        );

        for (auto& m : model) m *= scale;
        rb.SetModel(model);
        rb.SetColor(olc::Pixel(rand() % 256, rand() % 256, rand() % 256));

        scene.AddShape(rb);
        index++;

        is_preview = false;
    }

    void Logic() {

        if (df > sf) std::swap(sf, df);
        if (!is_preview) return;

        std::vector<olc::vf2d> rb_model = model;
        for (int i = 0; i < n; i++) {
            // Model scaling
            rb_model[i] *= scale;

            // Rotation
            vertices[i].x = rb_model[i].x * cosf(angle) - rb_model[i].y * sinf(angle);
            vertices[i].y = rb_model[i].x * sinf(angle) + rb_model[i].y * cosf(angle);

            // Scaling
            vertices[i] *= len;

            // Translation
            vertices[i] += pos;
        }
    }

    void PreviewRender(olc::PixelGameEngine* pge) {
        
        gui.run();

        if (!is_preview) return;

        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;

            pge->DrawLine(vertices[i], vertices[j]);
        }
    }
};

class Game : public olc::PixelGameEngine {
private:
    Scene scene;
    RigidBodyDraw rb_draw;
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

        scene = Scene(this);

        rb_draw.Initialize();

        scene.Initialize({ ScreenWidth() * 1.0f, ScreenHeight() * 1.0f });

        scene.AddShape({ ScreenWidth() * 0.5f, ScreenHeight() * 1.25f }, 4, ScreenWidth() * 0.5f, PI/4.0f, 0.0f, 0.0f);
        scene.AddShape({ ScreenWidth() * 0.5f, ScreenHeight() * 0.5f }, 5, 25.0f, PI/4.0f, 0.0f, 25.0f);
        scene.AddShape({ ScreenWidth() * 0.75f, ScreenHeight() * 0.25f }, 10, 10.0f, 0.0f, 0.0f, 10.0f, olc::YELLOW);

        Constraint rope({ ScreenWidth() * 0.25f, ScreenHeight() * 0.25f }, 60.0f, 1.5f, 0.2f, true);
        rope.Attach(scene.GetShape(1).GetID());
        scene.AddConstraint(rope);

        Constraint rope2({ ScreenWidth() * 0.75f, ScreenHeight() * 0.25f }, 20.0f, 1.5f, 0.9f, true);
        rope2.Attach(scene.GetShape(2).GetID());
        scene.AddConstraint(rope2);

        return true;
    }

    bool OnUserUpdate(float dt) override {

        const olc::vf2d& m_pos = GetMousePos() * 1.0f;

        // Input
        if (GetKey(olc::Z).bPressed) {
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

        bool is_shift = GetKey(olc::SHIFT).bHeld;

        int key = 0;
        if (GetKey(olc::A).bHeld) key = 0;
        else if (GetKey(olc::W).bHeld) key = 1;
        else if (GetKey(olc::D).bHeld) key = 2;
        else if (GetKey(olc::S).bHeld) key = 3;

        if (!is_shift) {
            if (GetMouse(1).bPressed) rb_draw.OnMousePress();
            if (GetMouse(1).bHeld) rb_draw.OnMouseInput(m_pos, Sign(GetMouseWheel()) * 5.0f * dt, key);
            if (GetMouse(1).bReleased) rb_draw.OnMouseRelease(scene);
        }

        if (is_shift && GetMouse(1).bHeld) {
            for (const auto& c : scene.GetConstraints()) {
                if (scene.GetShape(c.GetID()).IsContainPoint(GetMousePos() * 1.0f)) {
                    scene.GetShape(c.GetID()).SetInputState(true);
                    scene.GetShape(c.GetID()).SetPosition(GetMousePos() * 1.0f);
                }
            }
        }

        if (is_shift && GetMouse(1).bReleased) {

            const auto& constraints = scene.GetConstraints();

            for (int i = 0; i < constraints.size(); i++) {
                if (scene.GetShape(constraints[i].GetID()).GetInputState()) {
                    scene.GetShape(constraints[i].GetID()).SetInputState(false);
                    if (constraints[i].IsSling()) {
                        scene.GetConstraint(i).ApplyForces(scene.GetShape(constraints[i].GetID()), 1.0f, true);
                        //scene.GetConstraints()[i].Reset();
                    }
                }
            }
        }

        // if (rb_draw.index > 0) {
        //     scene.GetShape(rb_draw.index).SetPosition(m_pos);
        //     if (GetKey(olc::SPACE).bHeld) scene.GetShape(rb_draw.index).AddAngle(5.0f * dt);
        // }

        // Logic
        Clear(olc::BLACK);
        rb_draw.Logic();
        scene.Draw(this, true);
        for (int i = 0; i < n_iter; i++) scene.Update(dt, false);

        // Draw
        
        rb_draw.PreviewRender(this);

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