#pragma once
#include "olcPixelGameEngine.h"

// Some mathematical constants
const float PI = 3.1415926f;
const float g = 10.0f;
const float EPSILON = 0.00001f;

struct DebugManager {
    olc::PixelGameEngine* pge = nullptr;
} debug_mgr;

// Class Declarations
class RigidBody {
private:
    olc::vf2d position, velocity;
    std::vector<olc::vf2d> vertices, prev_vertices, model;
    float angle = 0.0f, len = 0.0f;
    int n = 0; // Vertices

    bool first_iter = false;

    float angular_velocity = 0.0f;
    float mass = 0.0f, inv_mass = 0.0f;
    float I = 0.0f, inv_I = 0.0f; // Moment of inertia
    float e = 0.1f; // Coefficient of restitution
    float sf = 0.8f, df = 0.4f; // Coefficient of static friction and dynamic friction

    int id = 0;
    bool is_input = false; // Is the rigid body held by mouse

    olc::Pixel color = olc::WHITE;
public:
    RigidBody() {}
    RigidBody(const olc::vf2d& p, int _n, float _len, float _angle, float a, float m, float _e = 0.1f, float _sf = 0.8f, float _df = 0.4f, int _id = 0);

    void Logic(float dt, bool is_debug = false);
    void ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact, float dt = 1.0f, float dir = 1.0f);

    bool IsConstrained(const olc::vf2d& a, const olc::vf2d& b); // Is the polygon within bounds
    bool IsContainPoint(const olc::vf2d& p); // Is the given point within polygon bounds

    void Draw(olc::PixelGameEngine* pge, bool is_fill = false, float alpha = 0.0f);
    
    static std::pair<float, olc::vf2d> SATOverlap(RigidBody& a, RigidBody& b);

    void SetModel(const std::vector<olc::vf2d>& m) { model = m; }
    void SetColor(const olc::Pixel& c) { color = c; }

    void Move(const olc::vf2d& o) { position += o; }
    void SetPosition(const olc::vf2d& p) { position = p; }
    const olc::vf2d& GetPosition() const { return position; }
    
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

    void AddVelocity(const olc::vf2d& _v) { velocity += _v; }
    void SetVelocity(const olc::vf2d& v) { velocity = v; }
    const olc::vf2d& GetVelocity() const { return velocity; }

    const float& GetLen() const { return len; }
    const float& GetRestitution() const { return e; }
    const float& GetFriction(int n) const { return !n ? sf : df; }

    const int& GetID() const { return id; }
    
    const bool& GetInputState() const { return is_input; }
    void SetInputState(bool state) { is_input = state; }
};

using Edge = std::tuple<olc::vf2d, olc::vf2d, olc::vf2d>;
class Manifold {
private:
    RigidBody* a = nullptr,
             * b = nullptr;

    float overlap = 0.0f;
    olc::vf2d normal;
    std::vector<olc::vf2d> points; // Contaxts
    std::vector<float> overlaps; // Overlaps

    float sf = 0.0f, df = 0.0f;
public:
    Manifold() {}
    Manifold(RigidBody* _a, RigidBody* _b, float _overlap, const olc::vf2d& _normal)
        : a(_a), b(_b), overlap(_overlap), normal(_normal) {
            sf = std::sqrt(a->GetFriction(0) * b->GetFriction(0));
            df = std::sqrt(a->GetFriction(1) * b->GetFriction(1));
        }
    
    Edge GetBestEdge(int index, float dir);
    std::pair<Edge, Edge> GetRefIncEdge();
    std::vector<olc::vf2d> GetContactPoints();
    std::vector<olc::vf2d> Clip(const olc::vf2d& v1, const olc::vf2d& v2, const olc::vf2d& n, float o);
    
    void ApplyForces(float dt);
    void PositionalCorrection();

    void Logic(float dt, int iter = 1, bool is_debug = false);
};

class Constraint {
private:
    olc::vf2d pivot_pos, point_pos;
    float len = 0.0f, k = 0.8f, b = 0.2f;

    int rb_id = -1;
    bool is_sling = false;
public:
    Constraint() {}
    Constraint(const olc::vf2d& p, float _len, float _k, float _b, bool _is_sling = false)
        : pivot_pos(p), len(_len), k(_k), b(_b), is_sling(_is_sling) {}

    void ApplyForces(RigidBody& rb, float dt, bool is_input = false);
    void Update(RigidBody& rb, float dt);

    void Draw(olc::PixelGameEngine* pge, olc::Pixel color = olc::WHITE);

    void SetPivotPosition(const olc::vf2d& p) { pivot_pos = p; }
    const olc::vf2d& GetPivotPosition() const { return pivot_pos; }

    void Attach(int id) { rb_id = id; }
    const int& GetID() const { return rb_id; }
    void Reset() { rb_id = -1; }

    const bool& IsSling() const { return is_sling; }
};

class Scene {
private:
    std::vector<RigidBody> shapes;
    std::vector<Constraint> constraints;

    float accumulator = 0.0f,
          delay       = 0.1f,
          inv_FPS     = 1.0f / 60.0f,
          alpha       = 0.0f;
    
    olc::vf2d screen_size;
public:
    Scene() {}
    Scene(olc::PixelGameEngine* pge) { debug_mgr.pge = pge; }
    void Initialize(const olc::vf2d& _screen_size) { screen_size = _screen_size; }

    void Update(float dt, bool is_debug = false);
    void Draw(olc::PixelGameEngine* pge, bool is_fill = false);

    void AddShape(const olc::vf2d& p, int n_vertices, float len, float angle, float angular_velocity, float mass, olc::Pixel color = olc::WHITE, float e = 0.1f, float sf = 0.8f, float df = 0.4f);
    void AddShape(const RigidBody& rb) { shapes.push_back(rb); }

    void AddConstraint(const olc::vf2d& p, float len, float k, float b, int index);
    void AddConstraint(const Constraint& c) { constraints.push_back(c); }

    std::vector<RigidBody>& GetShapes() { return shapes; }
    std::vector<Constraint>& GetConstraints() { return constraints; }

    RigidBody& GetShape(int index) { return shapes[index]; }
    Constraint& GetConstraint(int index) { return constraints[index]; }
};



// Class implementations
// RigidBody

RigidBody::RigidBody(const olc::vf2d& p, int _n, float _len, float _angle, float a, float m, float _e, float _sf, float _df, int _id) 
    : position(p), n(_n), len(_len), angle(_angle), angular_velocity(a), mass(m), e(_e), sf(_sf), df(_df), id(_id) {
    for (int i = 0; i < n; i++) {
            model.push_back({ cosf(2.0f * PI / n * i), sinf(2.0f * PI / n * i) });
        }
    vertices.resize(n);
    prev_vertices.resize(n);

    inv_mass = mass == 0.0f ? 0.0f : 1.0f / mass;
    I = mass * len * len / 12.0f;
    inv_I = I == 0.0f ? 0.0f : 1.0f / I;
}

void RigidBody::Logic(float dt, bool is_debug) {
    // Physics Logic
    if (!is_debug) {
        if (mass > 0.0f && !is_input) velocity.y += g * dt; // Gravity

        position += velocity * dt;
        angle += angular_velocity * dt;
    }

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

    if (!first_iter) { 
        first_iter = true;
        prev_vertices = vertices;
    }
}

void RigidBody::ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact, float dt, float dir) {
    velocity += impulse * inv_mass * dt; // v = (f/m) * dt
    angular_velocity += contact.cross(impulse) * inv_I * dt; // w = (restoring_torque / I) * dt
}

std::pair<float, olc::vf2d> RigidBody::SATOverlap(RigidBody& r1, RigidBody& r2) {
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
    if (displacement.dot(overlap_axis) < 0.0f) overlap_axis *= -1.0f;
    
    return { overlap, overlap_axis };
}

void RigidBody::Draw(olc::PixelGameEngine* pge, bool is_fill, float alpha) {
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

bool RigidBody::IsConstrained(const olc::vf2d& a, const olc::vf2d& b) {
    return !(position.x + len < a.x || position.y + len < a.y || position.x > b.x + len || position.y > b.y + len);
}

bool RigidBody::IsContainPoint(const olc::vf2d& p) {
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n,
            k = (i + 2) % n;

        const olc::vf2d& ab = vertices[j] - vertices[0],
                            bc = vertices[k] - vertices[j],
                            ca = vertices[0] - vertices[k];

        const olc::vf2d ap = vertices[0] - p,
                        bp = vertices[j] - p,
                        cp = vertices[k] - p;

        if (ab.cross(ap) <= 0.0f && bc.cross(bp) <= 0.0f && ca.cross(cp) <= 0.0f) return true;        
    }
    return false;
}

// Manifold

Edge Manifold::GetBestEdge(int index, float dir) {
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

    if (left_edge.dot(n) <= right_edge.dot(n)) {
        return { left_vertex, far_vertex, left_edge }; 
    }
    else {
        return { right_vertex, far_vertex, right_edge }; 
    }
}

std::pair<Edge, Edge> Manifold::GetRefIncEdge() {
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

std::vector<olc::vf2d> Manifold::Clip(const olc::vf2d& v1, const olc::vf2d& v2, const olc::vf2d& n, float o) {
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

std::vector<olc::vf2d> Manifold::GetContactPoints() {
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
    float p1 = std::fabs(cp2[0].dot(ref_norm)) - d,
          p2 = std::fabs(cp2[1].dot(ref_norm)) - d;
    std::vector<float> p = { p1, p2 };

    if      (p2 < 0.0f) { cp2.erase(cp2.begin() + 1); p.erase(p.begin() + 1); }
    else if (p1 < 0.0f) { cp2.erase(cp2.begin() + 0); p.erase(p.begin() + 0); }

    //for (auto& p : cp2) debug_mgr.pge->FillCircle(p, 5, olc::BLUE);

    points = cp2;
    overlaps = p;
    return cp2;
}

void Manifold::ApplyForces(float dt) {
    if (points.size() == 0) return;

    auto VectorProduct = [](float a, const olc::vf2d& v) {
        return olc::vf2d(v.y * a, v.x * -a);
    };

    for (const olc::vf2d& p : points) {

        const olc::vf2d& ra = (p - a->GetPosition()),
                         rb = (p - b->GetPosition());
    
        olc::vf2d va = VectorProduct(a->GetAngularVelocity(), ra),
                  vb = VectorProduct(b->GetAngularVelocity(), rb);

        olc::vf2d rv = (b->GetVelocity() - vb) - (a->GetVelocity() - va);
        
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
        j = std::fmax(j, EPSILON * 100.0f);

        // Normal resolution
        olc::vf2d impulse = j * normal;
        a->ApplyImpulse(-impulse, ra, 1.0f);
        b->ApplyImpulse( impulse, rb, 1.0f);
        
        va = VectorProduct(a->GetAngularVelocity(), ra);
        vb = VectorProduct(b->GetAngularVelocity(), rb);

        rv = (b->GetVelocity() - vb) - (a->GetVelocity() - va);
        rv_normal = rv.dot(normal);

        const olc::vf2d& t = (rv - rv_normal * normal).norm(); // Tangent from triangle law with rv and rv_normal vector
        float jt = -t.dot(rv) / inv_mass_sum;
        jt /= points.size();

        if (std::fabs(jt - 0.0f) <= EPSILON) { return; }

        olc::vf2d friction_impulse;
        if (std::fabs(jt) <= j * sf) { friction_impulse = jt * t; }
        else { friction_impulse = -j * df * t; }

        a->ApplyImpulse(-friction_impulse, ra, 1.0f);
        b->ApplyImpulse( friction_impulse, rb, 1.0f);
    }
}

void Manifold::PositionalCorrection() {
    if (overlaps.size() == 2) {
        
        // Get the average of the two overlaps (to avoid too much deviation)
        if (std::fabs(overlap - overlaps[1]) <= 1.0f) {
            overlap += overlaps[1];
            overlap /= (int)overlaps.size(); 
        } else {
            overlap = std::fmin(overlaps[0], overlaps[1]);
        }
    }
    
    float p = 0.5f;
    const olc::vf2d& direction = std::fmax(overlap - 0.05f, 0.0f) / (a->GetInvMass() + b->GetInvMass()) * normal * p;
    a->Move(-direction * a->GetInvMass());
    b->Move( direction * b->GetInvMass());
}

void Manifold::Logic(float dt, int iter, bool is_debug) {
    GetContactPoints();                                        // Get contact points
    if (!is_debug) {
        for (int i = 0; i < iter; i++) ApplyForces(dt);        // Dynamic resolution
        PositionalCorrection();                                // Static resolution
    }
}

// Constraint
// Apply restoring force on rope
void Constraint::ApplyForces(RigidBody& rb, float dt, bool is_input) {
    const olc::vf2d& direction = rb.GetPosition() - pivot_pos;        

    float extension = direction.mag();
    
    olc::vf2d dir = direction / extension;

    const olc::vf2d& offset = (extension - len) * dir;
    const olc::vf2d& force = -k * offset - b * rb.GetVelocity();

    float inv_mass = is_input ? 1.0f : rb.GetInvMass();

    if (!rb.GetInputState()) rb.AddVelocity(force * inv_mass * dt);
}

void Constraint::Update(RigidBody& rb, float dt) {

    if (rb_id < 0) return;
    olc::vf2d direction = rb.GetPosition() - pivot_pos;
    float dir_mag2 = direction.mag2();

    if (dir_mag2 > len * len) { ApplyForces(rb, dt); }
    point_pos = rb.GetPosition();
}

void Constraint::Draw(olc::PixelGameEngine* pge, olc::Pixel color) {
    pge->DrawLine(pivot_pos, point_pos, color);
}

// Scene

void Scene::Update(float dt, bool is_debug) {
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

        for (auto& m : manifolds) { m.Logic(dt, 5, is_debug); }
        
        std::vector<int> shapeID;
        for (auto& c : constraints) {
            c.Update(shapes[c.GetID()], inv_FPS);
            shapeID.push_back(c.GetID());
        }
        
        for (int i = shapes.size() - 1; i >= 0; i--) {
            shapes[i].Logic(inv_FPS, is_debug);

            if (std::find(shapeID.begin(), shapeID.end(), i) != shapeID.end()) continue;

            // Shapes in bounds
            if (!shapes[i].IsConstrained({ 0.0f, 0.0f }, { screen_size.x, screen_size.y })) { shapes.erase(shapes.begin() + i); }
        }

        for (int i = constraints.size() - 1; i >= 0; i--) {
            if (constraints[i].GetID() < 0) { constraints.erase(constraints.begin() + i); }
        }
    }

    alpha = accumulator / inv_FPS;
}

void Scene::Draw(olc::PixelGameEngine* pge, bool is_fill) {
    for (auto& s : shapes)      s.Draw(pge, is_fill, 0.0f);
    for (auto& c : constraints) c.Draw(pge);
}

void Scene::AddShape(const olc::vf2d& p, int n_vertices, float len, float angle, float angular_velocity, float mass, olc::Pixel color, float e, float sf, float df) {
    RigidBody rb(p, n_vertices, len, angle, angular_velocity, mass, e, sf, df, shapes.size());
    rb.SetColor(color);
    shapes.push_back(rb);
}

void Scene::AddConstraint(const olc::vf2d& p, float len, float k, float b, int index) {
    Constraint c(p, len, k, b);
    if (index >= 0) c.Attach(index);
    constraints.push_back(c);
}