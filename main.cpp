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
    
        gui.add_int_slider("vertices", "Vertex ", { 80, 5 }, { 100, 10 }, { 3, 10 }, &n);
        gui.add_float_slider("angular_velocity", "AngleVel ", { 80, 20 }, { 100, 10 }, { 0.0f, 5.0f }, &angular_velocity);
        gui.add_float_slider("mass", "Mass ", { 80, 35 }, { 100, 10 }, { 0.0f, 100.0f }, &mass);
        gui.add_float_slider("restitution", "Restitution ", { 80, 50 }, { 100, 10 }, { 0.0f, 2.0f }, &e);
        gui.add_float_slider("static_friction", "sFriction ", { 80, 65 }, { 100, 10 }, { 0.0f, 5.0f }, &sf);
        gui.add_float_slider("dynamic_friction", "dFriction ", { 80, 80 }, { 100, 10 }, { 0.0f, 5.0f }, &df);

        gui.find_element("vertices")->set_text_color(olc::Pixel(0, 255, 0));
        gui.find_element("angular_velocity")->set_text_color(olc::Pixel(0, 255, 50));
        gui.find_element("mass")->set_text_color(olc::Pixel(0, 255, 100));
        gui.find_element("restitution")->set_text_color(olc::Pixel(0, 255, 150));
        gui.find_element("static_friction")->set_text_color(olc::Pixel(0, 255, 200));
        gui.find_element("dynamic_friction")->set_text_color(olc::Pixel(0, 255, 255));
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
    int n_iter = 5;
    
    RigidBody* selected_shape = nullptr;
    Constraint* selected_constraint = nullptr;

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
        scene.AddShape({ 0.0f, 0.0f }, 10, 15.0f, 0.0f, 0.0f, 10.0f, olc::GREEN);

        Constraint c({ ScreenWidth() * 0.25f, ScreenHeight() * 0.25f }, 100.0f, 0.8f, 0.2f, false);
        c.Attach(1);
        scene.AddConstraint(c);

        return true;
    }

    bool OnUserUpdate(float dt) override {

        const olc::vf2d& m_pos = GetMousePos() * 1.0f;

        // Input
        if (GetKey(olc::Z).bPressed) {
            scene.AddShape(
                m_pos,                                                  // Position       
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

        if (GetMouse(0).bPressed) {
            for (auto& c : scene.GetConstraints()) {
                RigidBody& shape = scene.GetShape(c.GetID());
                if (shape.IsContainPoint(m_pos)) {
                    selected_shape = &shape;
                    selected_constraint = &c;
                    shape.is_input = true;
                }
            }
        }

        if (selected_constraint != nullptr) {
            if (GetMouse(0).bHeld) selected_shape->SetPosition(m_pos);

            else if (GetMouse(0).bReleased) {
                selected_shape->is_input = false;
                selected_constraint->ApplyForces(*selected_shape, 1.0f, true);
                selected_shape = nullptr;
                selected_constraint = nullptr;
            }
        }
        
        // Logic
        rb_draw.Logic();
        for (int i = 0; i < n_iter; i++) scene.Update(dt, false);

        // Draw
        Clear(olc::BLACK);
        scene.Draw(this, true);
        rb_draw.PreviewRender(this);

        return true;
    }
};

int main() {

    Game game;
    if (game.Construct(512, 512, 1, 1, false, true)) {
        game.Start();
    }

    return 0;
}