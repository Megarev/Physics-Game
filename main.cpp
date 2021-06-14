#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include <random>

const float PI = 3.1415926f;
const float g = 10.0f;

class RigidBody {
private:
    olc::vf2d position, velocity;
    std::vector<olc::vf2d> vertices, prev_vertices, model;
    float angle = 0.0f, len = 0.0f;
    int n = 0; // Vertices

    float angular_velocity = 0.0f;
    float mass = 0.0f, inv_mass = 0.0f;
    float I = 0.0f, inv_I = 0.0f; // Moment of inertia
    float e = 0.1f; // Coefficient of restitution
    float sf = 0.8f, df = 0.4f; // Coefficient of static friction and dynamic friction

    olc::Pixel color;
public:
    RigidBody() {}
    RigidBody(const olc::vf2d& p, int _n, float _len, float _angle, float a, float m, float _e = 0.1f, float _sf = 0.8f, float _df = 0.4f) 
        : position(p), n(_n), len(_len), angle(_angle), angular_velocity(a), mass(m), e(_e), sf(_sf), df(_df) {
        for (int i = 0; i < n; i++) {
            model.push_back({ cosf(2.0f * PI / n * i), sinf(2.0f * PI / n * i) });
        }
        vertices.resize(n);
        prev_vertices.resize(n);

        inv_mass = mass == 0.0f ? 0.0f : 1.0f / mass;
        I = mass * len * len / 12.0f;
        inv_I = I == 0.0f ? 0.0f : 1.0f / I;
    }

    void SetColor(const olc::Pixel& c) { color = c; }

    void Logic(float dt, bool is_debug = false) {
        // Physics Logic
        if (mass > 0.0f && !is_debug) velocity.y += g * dt; // Gravity

        position += velocity * dt;
        angle += angular_velocity * dt;

        //prev_vertices = vertices; // Update prev_vertices

        for (int i = 0; i < n; i++) {
            // Rotation
            vertices[i] = {
                model[i].x * cosf(angle) - model[i].y * sinf(angle),
                model[i].x * sinf(angle) + model[i].y * cosf(angle) 
            };

            // Scaling
            vertices[i] *= len;

            // Translation
            vertices[i] += position;
        }
    }

    void ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact, float dt) {
        velocity += impulse * inv_mass * dt; // v = (f/m) * dt
        angular_velocity += -contact.cross(impulse) * inv_I * dt; // w = (restoring_torque / I) * dt
    }

    static std::pair<float, olc::vf2d> SATOverlap(RigidBody& r1, RigidBody& r2) {
        RigidBody* p1 = &r1, *p2 = &r2;

        float overlap = +INFINITY;
        olc::vf2d overlap_axis;

        bool is_overlap = false;

        for (int n = 0; n < 2; n++) {
            if (n == 1) std::swap(p1, p2);

            for (int i = 0; i < p1->n; i++) {
                int j = (i + 1) % p1->n;

                olc::vf2d edge_normal = (p1->vertices[j] - p1->vertices[i]).perp().norm();

                float min_r1 = +INFINITY, max_r1 = -INFINITY;

                for (const olc::vf2d& p : p1->vertices) {
                    float q = edge_normal.dot(p);
                    min_r1 = std::fmin(min_r1, q);
                    max_r1 = std::fmax(max_r1, q);
                }

                float min_r2 = +INFINITY, max_r2 = -INFINITY;

                for (const olc::vf2d& p : p2->vertices) {
                    float q = edge_normal.dot(p);
                    min_r2 = std::fmin(min_r2, q);
                    max_r2 = std::fmax(max_r2, q);
                }

                if (!(max_r2 >= min_r1 && max_r1 >= min_r2)) return { 0.0f, { 0.0f, 0.0f } };
                
                float new_overlap = std::fmin(max_r1, max_r2) - std::fmax(min_r1, min_r2);
                if (new_overlap < overlap) {
                    overlap = new_overlap;
                    overlap_axis = edge_normal;
                    is_overlap = true;
                }
            }    
        }

        olc::vf2d displacement = r2.GetPosition() - r1.GetPosition();
        if (displacement.dot(overlap_axis) <= 0.0f) overlap_axis *= -1.0f;
        
        return { overlap, overlap_axis };
    }

    void Draw(olc::PixelGameEngine* pge, bool is_fill = false, float alpha = 0.0f) {
        // Linearly interpolate position with respect to alpha
        std::vector<olc::vf2d> interpolated_vertices(n);
        for (uint32_t a = 0; a < vertices.size(); a++) {
            interpolated_vertices[a] = alpha * prev_vertices[a] + (1.0f - alpha) * vertices[a];
        }
        
        if (!is_fill) {
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                
                pge->DrawLine(interpolated_vertices[i], interpolated_vertices[j], color);
            }
        } else {
            for (int i = 0; i < n - 2; i++) {
                int j = (i + 1) % n,
                    k = (i + 2) % n;
            
                pge->FillTriangle(interpolated_vertices[0], interpolated_vertices[j], interpolated_vertices[k], color);
            }
        }
        prev_vertices = vertices; // Update vertices
    }

    bool IsConstrained(const olc::vf2d& a, const olc::vf2d& b) {
        return !(
            position.x + len < a.x || position.y + len < a.y || position.x > b.x + len || position.y > b.y + len
        );
    }

    void SetPosition(const olc::vf2d& p) { position = p; }
    const olc::vf2d& GetPosition() const { return position; }
    void Move(const olc::vf2d& o) { position += o; }
    
    void AddAngle(float a) { angle += a; }
    const float& GetAngle() const { return angle; }

    const int& GetN() const { return n; }
    const olc::vf2d& GetVertex(int i) const { return vertices[i]; }

    const float& GetMass() const { return mass; }
    const float& GetInvMass() const { return inv_mass; }

    const float& GetInertia() const { return I; }
    const float& GetInvInertia() const { return inv_I; }

    void SetAngularVelocity(float a) { angular_velocity = a; }
    const float& GetAngularVelocity() const { return angular_velocity; }

    void SetVelocity(const olc::vf2d& v) { velocity = v; }
    const olc::vf2d& GetVelocity() const { return velocity; }

    const float& GetLen() const { return len; }
    const float& GetRestitution() const { return e; }

    const float& GetFriction(int n) const { return !n ? sf : df; } 
};

using Edge = std::tuple<olc::vf2d, olc::vf2d, olc::vf2d>;
class Manifold {
private:
    RigidBody* a = nullptr,
             * b = nullptr;

    float overlap = 0.0f;
    olc::vf2d normal;
    std::vector<olc::vf2d> points; // Contact points

    Edge GetBestEdge(int index, float dir) {
        RigidBody* r = !index ? a : b;
        const olc::vf2d& n = normal * dir;

        float m_dot = -INFINITY;
        int vertex = -1;

        for (int i = 0; i < r->GetN(); i++) {
            float distance = r->GetVertex(i).dot(n);
            if (distance > m_dot) {
                m_dot = distance;
                vertex = i;
            }
        }

        olc::vf2d far_vertex = r->GetVertex(vertex),
                  left_vertex = r->GetVertex(vertex-1 < 0 ? r->GetN()-1 : vertex-1),
                  right_vertex = r->GetVertex((vertex+1) % r->GetN());

        olc::vf2d left_edge = (far_vertex - left_vertex),
                  right_edge = (far_vertex - right_vertex);
    
        if (left_edge.dot(n) <= right_edge.dot(n)) { return { left_vertex, far_vertex, left_edge }; }
        else { return { right_vertex, far_vertex, right_edge }; }
    }

    std::vector<olc::vf2d> Clip(const olc::vf2d& v1, const olc::vf2d& v2, const olc::vf2d& n, float o) {
        std::vector<olc::vf2d> cp;
        float d1 = n.dot(v1) - o, // Signed distance between ref vertex and v1
              d2 = n.dot(v2) - o; // Signed distance between ref vertex and v2
        
        if (d1 >= 0.0f) cp.push_back(v1);
        if (d2 >= 0.0f) cp.push_back(v2);

        if (d1 * d2 < 0.0f) {
            // Linearly interpolate ref edge
            float u = d1 / (d1 - d2);

            cp.push_back(v1 + u * (v2 - v1));
        }
        
        return cp;
    }
public:
    std::pair<Edge, Edge> GetRefIncEdge() {

        const auto& edge_a = GetBestEdge(0, 1.0f);
        const auto& edge_b = GetBestEdge(1,-1.0f);

        if ((std::get<2>(edge_a).dot(normal)) >= (std::get<2>(edge_b).dot(normal))) {
            // Reference edge is A
            return { edge_a, edge_b };
        } else {
            // Reference edge is B
            return { edge_b, edge_a };
        }
    }

    std::vector<olc::vf2d> GetContactPoints() {
        const auto& data = GetRefIncEdge();
        const olc::vf2d& ref_edge = std::get<2>(data.first).norm();
        
        float o1 = ref_edge.dot(std::get<0>(data.first)); // Distance from origin to first vertex along ref edge
        auto cp1 = Clip(std::get<0>(data.second), std::get<1>(data.second), ref_edge, o1);
        if (cp1.size() != 2) return { { 0.0f, 0.0f } };

        float o2 = ref_edge.dot(std::get<1>(data.first)); // Distance from origin to second vertex along ref edge
        auto cp2 = Clip(cp1[0], cp1[1], -ref_edge, -o2);
        if (cp2.size() != 2) return { { 0.0f, 0.0f } };

        const olc::vf2d& ref_norm = ref_edge.perp();

        const olc::vf2d& m_vertex = std::get<1>(data.first);

        float d = std::fabs(ref_norm.dot(m_vertex));
        if (std::fabs(cp2[1].dot(ref_norm)) < (d)) cp2.erase(cp2.begin() + 1);
        if (std::fabs(cp2[0].dot(ref_norm)) < (d)) cp2.erase(cp2.begin() + 0);

        points = cp2;
        return cp2;
    }

    Manifold() {}
    Manifold(RigidBody* _a, RigidBody* _b, float _overlap, const olc::vf2d& _normal)
        : a(_a), b(_b), overlap(_overlap), normal(_normal) {}
    
    void ApplyForces(float dt) {
        if (points.size() == 0) return;

        auto VectorProduct = [](float a, const olc::vf2d& v) {
            return olc::vf2d(v.y * a, v.x * -a);
        };

        for (const olc::vf2d& p : points) {
            const olc::vf2d& ra = (a->GetPosition() - p),
                             rb = (b->GetPosition() - p);
        
            olc::vf2d va = VectorProduct(a->GetAngularVelocity(), ra),
                      vb = VectorProduct(b->GetAngularVelocity(), rb);

            olc::vf2d rv = (b->GetVelocity() + vb) - (a->GetVelocity() + va);
            
            float rv_normal = rv.dot(normal);
            if (rv_normal > 0.0f) return;

            float ra_n = ra.cross(normal),
                  rb_n = rb.cross(normal);

            float inv_mass_sum = (
                a->GetInvMass() + ra_n*ra_n*a->GetInvInertia() + 
                b->GetInvMass() + rb_n*rb_n*b->GetInvInertia()
            );

            float e = std::fmin(a->GetRestitution(), b->GetRestitution());
            float j = -(1.0f + e) * rv_normal / inv_mass_sum;
            j /= points.size();

            // Normal resolution
            olc::vf2d impulse = j * normal;
            a->ApplyImpulse(-impulse, ra, 1.0f);
            b->ApplyImpulse( impulse, rb, 1.0f);
        
            va = VectorProduct(a->GetAngularVelocity(), ra);
            vb = VectorProduct(b->GetAngularVelocity(), rb);

            rv = (b->GetVelocity() + vb) - (a->GetVelocity() + va);
            rv_normal = rv.dot(normal);

            olc::vf2d t = (rv - rv_normal * normal).norm(); // Tangent from triangle law with rv and rv_normal vector
            float jt = -t.dot(rv) / inv_mass_sum;
            jt /= points.size();

            if (std::fabs(jt) <= 0.1f) return;

            float sf = 0.5f * (a->GetFriction(0) + b->GetFriction(0)),
                  df = 0.5f * (a->GetFriction(1) + b->GetFriction(1));
            
            olc::vf2d friction_impulse;
            if (std::fabs(jt) < j * sf) { friction_impulse = jt * t; }
            else { friction_impulse = -j * df * t; }

            a->ApplyImpulse(-friction_impulse, ra, 1.0f);
            b->ApplyImpulse( friction_impulse, rb, 1.0f);
        }
    }

    void PositionalCorrection() {
        float p = 0.4f;
        olc::vf2d direction = std::fmax(overlap - 0.05f, 0.0f) / (a->GetInvMass() + b->GetInvMass()) * p * normal;
        a->Move(-direction * a->GetInvMass());
        b->Move( direction * b->GetInvMass());
    }
};

class Scene {
private:
    std::vector<RigidBody> shapes;
    float accumulator = 0.0f;

    olc::vf2d screen_size;
public:
    Scene() {}

    void SetBounds(const olc::vi2d& _screen_size) { screen_size = (olc::vf2d)_screen_size; }

    void Update(float dt, olc::PixelGameEngine* pge) {

        // Simulation parameters
        float FPS = 60.0f,
              delay = 0.1f;

        float inv_FPS = 1.0f / FPS;

        std::vector<Manifold> manifolds;

        accumulator = std::fmin(accumulator + dt, delay);

        if (accumulator > inv_FPS) {
            accumulator -= inv_FPS;

            std::vector<Manifold> manifolds;
            for (uint32_t a = 0; a < shapes.size() - 1; a++) {
                for (uint32_t b = a + 1; b < shapes.size(); b++) {
                    const auto& manifold_data = RigidBody::SATOverlap(shapes[a], shapes[b]);
                    if (manifold_data.first > 0.0f) {
                        // Add to vector if overlap > 0
                        manifolds.push_back(Manifold(&shapes[a], &shapes[b], manifold_data.first, manifold_data.second));
                    }
                }
            }

            for (auto& m : manifolds) {
                const auto& edge_data = m.GetRefIncEdge();
                const auto& cp = m.GetContactPoints();

                m.ApplyForces(inv_FPS);
                m.PositionalCorrection();
            }

            for (int i = shapes.size() - 1; i >= 0; i--) {
                shapes[i].Logic(inv_FPS);
                if (!shapes[i].IsConstrained({ 0.0f, 0.0f }, { screen_size.x, screen_size.y })) {
                    shapes.erase(shapes.begin() + i);
                }
            }
        }
    }

    void Draw(olc::PixelGameEngine* pge, bool is_fill = false, float alpha = 0.0f) {
        for (auto& s : shapes) s.Draw(pge, is_fill, alpha);
    }

    void AddShape(const olc::vf2d& p, int n_vertices, float len, float angle, float angular_velocity, float mass, olc::Pixel color = olc::WHITE, float e = 0.1f, float sf = 0.8f, float df = 0.4f) {
        shapes.push_back(RigidBody(p, n_vertices, len, angle, angular_velocity, mass, e, sf, df));
        shapes.back().SetColor(color);
    }

    const std::vector<RigidBody>& GetShapes() const { return shapes; }
};

class Game : public olc::PixelGameEngine {
private:
    Scene scene;

    static float Random(float a, float b) {
        std::random_device rd;
        std::mt19937 m(rd());
        std::uniform_real_distribution<> dist(a, b);

        return dist(m);
    };
public:
    Game() {
        sAppName = "Physics Game";
    }

    bool OnUserCreate() override {

        scene.SetBounds({ ScreenWidth(), ScreenHeight() });

        scene.AddShape({ ScreenWidth() * 0.5f, ScreenHeight() * 1.25f }, 4, ScreenWidth() * 0.5f, PI/4.0f, 0.0f, 0.0f);

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
                Random(-1.0f, 1.0f),                                    // Angular velocity
                Random( 2.0f, 10.0f),                                   // Mass
                olc::Pixel(rand() % 256, rand() % 256, rand() % 256),   // Color
                0.2f,                                                   // Coefficient of restitution
                0.2f,                                                   // Coefficient of static friction
                0.1f                                                    // Coefficient of dynamic friction
            );
        }

        // Logic
        scene.Update(dt, this);

        // Draw
        Clear(olc::BLACK);
        scene.Draw(this, true);
        
        return true;
    }
};

int main() {

    Game game;
    if (game.Construct(256, 256, 2, 2)) {
        game.Start();
    }

    return 0;
}