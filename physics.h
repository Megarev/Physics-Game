#pragma once
#include "olcPixelGameEngine.h"

// Some mathematical constants
constexpr float PI = 3.1415926f;
constexpr float g = 10.0f;
constexpr float EPSILON = 0.0001f;

olc::vf2d pixel_size = { 0.5f, 1.0f }; // For decal tileset

// Class Declarations
class RigidBody {
private:
    olc::vf2d position, prev_pos, velocity, acceleration;
    std::vector<olc::vf2d> vertices, prev_vertices, model, decal_model;
    bool first_iter = false;
    int id = 0;

    olc::Pixel color = olc::WHITE, shade = olc::WHITE;
public:
    int n = 0; // Vertices
    float angular_velocity = 0.0f, angular_acceleration = 0.0f;
    float mass = 0.0f, inv_mass = 0.0f;
    float I = 0.0f, inv_I = 0.0f; // Moment of inertia
    float e = 0.1f; // Coefficient of restitution
    float sf = 0.8f, df = 0.4f; // Coefficient of static friction and dynamic friction
    float len = 0.0f;
    bool is_move = true;
    int n_iter = 0, n_iter_update = 10;
    bool is_collide = false;
    float angle = 0.0f, prev_angle = 0.0f;

    olc::Decal* decal = nullptr;
    olc::vf2d offset = { 0.0f, 0.0f };

    bool is_input = false; // Is the rigid body held by mouse
public:
    RigidBody() {}
    RigidBody(const olc::vf2d& p, int _n, float _len, float _angle, float a, float m, float _e = 0.1f, float _sf = 0.8f, float _df = 0.4f, int _id = 0);

    void Logic(float dt, float alpha = 0.0f, bool is_debug = false);
    void ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact);

    bool IsConstrained(const olc::vf2d& a, const olc::vf2d& b); // Is the polygon within bounds
    bool IsContainPoint(const olc::vf2d& p); // Is the given point within polygon bounds

    void Draw(olc::PixelGameEngine* pge, bool is_fill = false, float alpha = 0.0f); // Polygon Line Drawing
    void Draw(olc::PixelGameEngine* pge, float alpha = 0.0f); // Decal drawing

    static std::pair<float, olc::vf2d> SATOverlap(RigidBody& a, RigidBody& b);

    void ApplyForce(const olc::vf2d& force, float dt) { if (mass > 0.0f) velocity += force * inv_mass * dt; }

    void SetModel(const std::vector<olc::vf2d>& m) { model = m; }
    void SetColor(const olc::Pixel& c) { color = c; }

    void Move(const olc::vf2d& o) { position += o; }
    void SetPosition(const olc::vf2d& p) { position = p; }
    const olc::vf2d& GetPosition() const { return position; }

    void SetPrevPosition(const olc::vf2d& p) { prev_pos = p; }
    const olc::vf2d& GetPrevPosition() const { return prev_pos; }

    void AddVelocity(const olc::vf2d& v) { velocity += v; }
    void SetVelocity(const olc::vf2d& v) { velocity = v; }
    const olc::vf2d& GetVelocity() const { return velocity; }

    void SetAcceleration(const olc::vf2d& a) { acceleration = a; }
    void AddAcceleration(const olc::vf2d& a) { acceleration += a; }
    const olc::vf2d& GetAcceleration() const { return acceleration; }

    const int& GetID() const { return id; }
    const olc::vf2d& GetVertex(int i) const { return vertices[i]; }
};

struct Edge { olc::vf2d v1, v2, edge, farthest, poly_pos; };

class Manifold {
private:
    RigidBody* a = nullptr, *b = nullptr;

    float overlap = 0.0f;
    olc::vf2d normal;
    std::vector<olc::vf2d> points; // Contacts
    std::vector<float> overlaps; // Overlaps
    Edge ref, inc;

    float sf = 0.0f, df = 0.0f;
public:
    Manifold() {}
    Manifold(RigidBody* _a, RigidBody* _b, float _overlap, const olc::vf2d& _normal)
        : a(_a), b(_b), overlap(_overlap), normal(_normal) {
        sf = std::sqrt(a->sf * b->sf);
        df = std::sqrt(a->df * b->df);
    }

    Edge GetBestEdge(int index, float dir);
    void GetRefIncEdge();
    void GetContactPoints();
    std::vector<olc::vf2d> Clip(const olc::vf2d& v1, const olc::vf2d& v2, const olc::vf2d& n, float o);

    void SetState();

    void ApplyForces(float dt);
    void PositionalCorrection();

    void Logic(float dt, int iter = 1, bool is_debug = false);
};

class Segment {
public:
    olc::vf2d a, b;
    float angle = 0.0f, len = 0.0f, len0 = 0.0f;
public:
    Segment() {}
    Segment(const olc::vf2d& _a, float _angle, float _len) 
        : a(_a), angle(_angle), len(_len), len0(_len) {}

    void Logic();
    void PointTo(const olc::vf2d& p);
    void Draw(olc::PixelGameEngine* pge) const;
};

class Constraint {
private:
    olc::vf2d pivot_pos, point_pos;
    float len = 0.0f, k = 0.8f, b = 0.2f;

    std::vector<Segment> segments;

    int rb_id = -1, n = 0;
    bool is_sling = false;
public:
    Constraint() {}
    Constraint(const olc::vf2d& p, float _len, float _k, float _b, int _n, bool _is_sling = false);

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
        delay = 0.1f,
        inv_FPS = 1.0f / 60.0f,
        alpha = 0.0f;

    olc::vf2d screen_size;
public:
    bool is_gravity = true;
    Scene() {}
    void Initialize(const olc::vf2d& _screen_size) { screen_size = _screen_size; }

    void Update(float dt, bool is_debug = false);
    void Draw(olc::PixelGameEngine* pge, bool is_fill = false);
    void DrawDecals(olc::PixelGameEngine* pge);

    void AddShape(const olc::vf2d& p, int n_vertices, float len, float angle, float angular_velocity, float mass, olc::Pixel color = olc::WHITE, float e = 0.1f, float sf = 0.8f, float df = 0.4f);
    void AddShape(const RigidBody& rb) { shapes.push_back(rb); }

    void AddConstraint(const olc::vf2d& p, float len, float k, float b, int n, int index);
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
        decal_model.push_back({ cosf(2.0f * PI / n * i + PI/4.0f) + 1.0f, sinf(2.0f * PI / n * i + PI/4.0f) + 1.0f });
        decal_model[i] /= 2.0f;
        decal_model[i] *= pixel_size;
    }
    vertices.resize(n);
    prev_vertices.resize(n);

    shade = olc::Pixel(rand() % 256, rand() % 256, rand() % 256);

    inv_mass = mass == 0.0f ? 0.0f : 1.0f / mass;
    I = mass * len * len / 12.0f;
    inv_I = I == 0.0f ? 0.0f : 1.0f / I;
}

void RigidBody::Logic(float dt, float alpha, bool is_debug) {
    // Physics Logic
    if (!is_debug && is_move) {
        velocity += acceleration * dt;
        angular_velocity += angular_acceleration * dt;

        position += velocity * dt;
        angle += angular_velocity * dt;
    }

    acceleration = { 0.0f, 0.0f };
    angular_acceleration = 0.0f;

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
        prev_pos = position;
        prev_angle = angle;
    }
}

void RigidBody::ApplyImpulse(const olc::vf2d& impulse, const olc::vf2d& contact) {
    velocity += impulse * inv_mass * 1.0f; // v = (f/m) * dt
    angular_velocity += contact.cross(impulse) * inv_I * 1.0f; // w = (restoring_torque / I) * dt
}

std::pair<float, olc::vf2d> RigidBody::SATOverlap(RigidBody& r1, RigidBody& r2) {
    RigidBody* p1 = &r1, * p2 = &r2;

    float overlap = +INFINITY;
    olc::vf2d overlap_axis;

    bool is_overlap = false;

    for (int n = 0; n < 2; n++) {
        if (n == 1) std::swap(p1, p2);

        for (int i = 0; i < p1->n; i++) {
            int j = (i + 1) % p1->n;

            olc::vf2d edge_normal = (p1->vertices[j] - p1->vertices[i]).perp();

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

    float overlap_len = overlap_axis.mag();
    overlap_axis /= overlap_len;
    overlap /= overlap_len;

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

            pge->DrawLine(interpolated_vertices[i], interpolated_vertices[j], color * 1.2f);
        }
    }
    else {
        for (int i = 0; i < n - 2; i++) {
            int j = (i + 1) % n,
                k = (i + 2) % n;

            pge->FillTriangle(interpolated_vertices[0], interpolated_vertices[j], interpolated_vertices[k], color);
        }
    }
    prev_vertices = vertices; // Update vertices
}

void RigidBody::Draw(olc::PixelGameEngine* pge, float alpha) {

    if (decal == nullptr) return;

    std::vector<olc::vf2d> interpolated_vertices(n), decal_draw_model(n);
    for (uint32_t a = 0; a < vertices.size(); a++) {
        interpolated_vertices[a] = alpha * prev_vertices[a] + (1.0f - alpha) * vertices[a];
        decal_draw_model[a] = decal_model[a] + offset;
    }

    pge->DrawPolygonDecal(decal, interpolated_vertices, decal_draw_model, shade);
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

    for (int i = 0; i < r->n; i++) {
        float distance = r->GetVertex(i).dot(n);
        if (distance > m_dot) {
            m_dot = distance;
            vertex = i;
        }
    }

    olc::vf2d far_vertex = r->GetVertex(vertex),
        right_vertex = r->GetVertex(vertex - 1 < 0 ? r->n - 1 : vertex - 1),
        left_vertex = r->GetVertex((vertex + 1) % r->n);

    olc::vf2d left_edge = (far_vertex - left_vertex),
        right_edge = (far_vertex - right_vertex);

    if (left_edge.dot(n) <= right_edge.dot(n)) {
        return { left_vertex, far_vertex, left_edge, far_vertex, r->GetPosition() };
    }
    else {
        return { right_vertex, far_vertex, right_edge, far_vertex, r->GetPosition() };
    }
}

void Manifold::GetRefIncEdge() {
    const auto& edge_a = GetBestEdge(0, 1.0f);
    const auto& edge_b = GetBestEdge(1, -1.0f);

    if (std::fabsf(edge_a.edge.dot(normal)) <= std::fabsf(edge_b.edge.dot(normal))) {
        // Reference edge is A
        ref = edge_a;
        inc = edge_b;
    }
    else {
        // Reference edge is B
        ref = edge_b;
        inc = edge_a;
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
        cp.push_back(v1 + d1 / (d1 - d2) * (v2 - v1));
    }

    return cp;
}

void Manifold::SetState() {
    
    size_t n_contacts = points.size();
    
    // First polygon, A
    if (a->n_iter++ > a->n_iter_update) {
        a->n_iter -= a->n_iter_update;

        bool is_linear_move = true;
        if ((a->GetPosition() - a->GetPrevPosition()).mag2() <= 0.01f) {
            is_linear_move = false;
            a->SetVelocity(a->GetVelocity() * 0.9f);
            if (a->GetVelocity().mag2() <= EPSILON) a->SetVelocity({ 0.0f, 0.0f });
        }

        bool is_rotational_move = true;
        if (std::fabsf(a->angle - a->prev_angle) <= 0.01f) {
            is_rotational_move = false;
            a->angular_velocity *= 0.9f;
            if (std::fabsf(a->angular_velocity) <= EPSILON) a->angular_velocity = 0.0f;
        }

        if (!(is_linear_move || is_rotational_move) && n_contacts >= 2) a->is_move = false;
        else a->is_move = true;

        a->prev_angle = a->angle;
        a->SetPrevPosition(a->GetPosition());
    }

    // Second polygon, B
    if (b->n_iter++ > b->n_iter_update) {
        b->n_iter -= b->n_iter_update;

        bool is_linear_move = true;
        if ((b->GetPosition() - b->GetPrevPosition()).mag2() <= 0.01f) {
            is_linear_move = false;
            b->SetVelocity(b->GetVelocity() * 0.9f);
            if (b->GetVelocity().mag2() <= EPSILON) b->SetVelocity({ 0.0f, 0.0f });
        }

        bool is_rotational_move = true;
        if (std::fabsf(b->angle - b->prev_angle) <= 0.01f) {
            is_rotational_move = false;
            b->angular_velocity *= 0.9f;
            if (std::fabsf(b->angular_velocity) <= EPSILON) b->angular_velocity = 0.0f;
        }

        if (!(is_linear_move || is_rotational_move) && n_contacts >= 2) b->is_move = false;
        else b->is_move = true;

        b->prev_angle = b->angle;
        b->SetPrevPosition(b->GetPosition());
    }
}

void Manifold::GetContactPoints() {
    GetRefIncEdge();
    const olc::vf2d& ref_edge = ref.edge.norm();

    auto cp1 = Clip(inc.v1, inc.v2, ref_edge, ref_edge.dot(ref.v1));
    if (cp1.size() < 2) return;

    auto cp2 = Clip(cp1[0], cp1[1], -ref_edge, -ref_edge.dot(ref.v2));
    if (cp2.size() < 2) return;

    olc::vf2d ref_norm = ref_edge.perp();
    if ((ref.poly_pos - inc.poly_pos).dot(ref_norm) < 0.0f) ref_norm *= -1.0f;

    float d = ref_norm.dot(ref.farthest);

    if (cp2[1].dot(ref_norm) - d < 0.0f) { cp2.erase(cp2.begin() + 1); }
    else if (cp2[0].dot(ref_norm) - d < 0.0f) { cp2.erase(cp2.begin() + 0); }

    points = cp2;
    for (const olc::vf2d& p : cp2) {
        overlaps.push_back(p.dot(ref_norm) - d);
    }
}

void Manifold::ApplyForces(float dt) {
    if (points.size() == 0) return;

    auto VectorProduct = [](float a, const olc::vf2d& v) { return olc::vf2d(v.y * a, v.x * -a); };

    for (const olc::vf2d& p : points) {

        const olc::vf2d& ra = (p - a->GetPosition()),
            rb = (p - b->GetPosition());

        olc::vf2d va = VectorProduct(a->angular_velocity, ra),
            vb = VectorProduct(b->angular_velocity, rb);

        olc::vf2d rv = (b->GetVelocity() - vb) - (a->GetVelocity() - va);

        float rv_normal = rv.dot(normal);
        if (rv_normal > 0.0f) return;

        float ra_n = ra.cross(normal),
            rb_n = rb.cross(normal);

        float inv_mass_sum = (
            a->inv_mass + ra_n * ra_n * a->inv_I +
            b->inv_mass + rb_n * rb_n * b->inv_I
            );

        float e = std::fminf(a->e, b->e);
        float j = -(1.0f + e) * rv_normal / inv_mass_sum;
        j /= points.size();

        if (j <= EPSILON) return;

        // Normal resolution
        olc::vf2d impulse = j * normal;
        a->ApplyImpulse(-impulse, ra);
        b->ApplyImpulse(impulse, rb);

        va = VectorProduct(a->angular_velocity, ra);
        vb = VectorProduct(b->angular_velocity, rb);

        rv = (b->GetVelocity() - vb) - (a->GetVelocity() - va);
        rv_normal = rv.dot(normal);
        
        olc::vf2d t = (rv - rv_normal * normal).norm(); // Tangent from triangle law with rv and rv_normal vector

        float jt = -t.dot(rv) / inv_mass_sum;
        jt /= points.size();

        olc::vf2d friction_impulse;
        if (std::fabsf(jt) <= j * sf) { friction_impulse = jt * t; }
        else { friction_impulse = -j * df * t; }

        a->ApplyImpulse(-friction_impulse, ra);
        b->ApplyImpulse(friction_impulse, rb);
    }
}

void Manifold::PositionalCorrection() {
    float p = 0.5f;
    const olc::vf2d& direction = std::fmaxf(overlap - 0.025f, 0.0f) * normal * p;

    float res = 1.0f; // Resolution percentage
    if (a->mass > 0.0f && b->mass > 0.0f) res = 0.5f;
    if (a->mass > 0.0f) a->Move(-direction * res);
    if (b->mass > 0.0f) b->Move(direction * res);
}

void Manifold::Logic(float dt, int iter, bool is_debug) {
    GetContactPoints();                                   // Get contact points
    if (!is_debug) {
        PositionalCorrection();                           // Static resolution
        for (int i = 0; i < iter; i++) ApplyForces(dt);   // Dynamic resolution
    }
    SetState();
}

// Segment
void Segment::Logic() {
    b = a + len * olc::vf2d(cosf(angle), sinf(angle));
}

void Segment::PointTo(const olc::vf2d& p) {
    const olc::vf2d& dir = p - a;
    angle = atan2f(dir.y, dir.x);

    a = p - len0 * olc::vf2d(cosf(angle), sinf(angle));
}

void Segment::Draw(olc::PixelGameEngine* pge) const {
    pge->DrawLine(a, b);
}

Constraint::Constraint(const olc::vf2d& p, float _len, float _k, float _b, int _n, bool _is_sling) 
    : pivot_pos(p), len(_len), k(_k), b(_b), n(_n), is_sling(_is_sling) {
    float segment_len = len / n;
    for (int i = 0; i < n; i++) segments.push_back(Segment({ 0.0f, 0.0f }, 0.0f, segment_len));
}

// Constraint
// Apply restoring force on rope
void Constraint::ApplyForces(RigidBody& rb, float dt, bool is_input) {
    const olc::vf2d& direction = rb.GetPosition() - pivot_pos;

    float extension = direction.mag();

    olc::vf2d dir = direction / extension;

    const olc::vf2d& offset = (extension - len) * dir;
    const olc::vf2d& force = -k * offset - b * rb.GetVelocity();

    float inv_mass = is_input ? 1.0f : rb.inv_mass;

    if (!rb.is_input) rb.AddAcceleration(force * inv_mass);
}

void Constraint::Update(RigidBody& rb, float dt) {

    if (rb_id < 0) return;
    olc::vf2d direction = rb.GetPosition() - pivot_pos;
    float dir_mag2 = direction.mag2();

    float offset = std::sqrtf(dir_mag2) - len;
    
    if (offset > 0.0f) {
        float dl = offset / n;
        for (int i = 0; i < segments.size(); i++) {
            segments[i].len = segments[i].len0 + dl;
            if (i < segments.size() - 1) segments[i].angle = segments.back().angle;
        }
    }

    if (dir_mag2 > len * len) { ApplyForces(rb, dt); }
    segments.back().PointTo(rb.GetPosition());
    for (int i = segments.size() - 2; i >= 0; i--) segments[i].PointTo(segments[i + 1].a);

    segments[0].a = pivot_pos; segments[0].Logic();
    for (int i = 1; i < segments.size(); i++) {
        segments[i].a = segments[i - 1].b;
        segments[i].Logic();
    }
}

void Constraint::Draw(olc::PixelGameEngine* pge, olc::Pixel color) {
    for (const auto& s : segments) s.Draw(pge);
}

// Scene

void Scene::Update(float dt, bool is_debug) {
    // Simulation parameters
    accumulator = std::fmin(accumulator + dt, delay);

    if (accumulator > inv_FPS) {
        accumulator -= inv_FPS;

        // Get all polygons which are constrained
        std::vector<int> shapeID;
        for (int i = constraints.size() - 1; i >= 0; i--) {
            constraints[i].Update(shapes[constraints[i].GetID()], inv_FPS);
            shapeID.push_back(constraints[i].GetID());
            if (constraints[i].GetID() < 0) { constraints.erase(constraints.begin() + i); }
        }

        for (int i = shapes.size() - 1; i >= 0; i--) {
            if (shapes[i].mass > 0.0f && is_gravity &&
                shapes[i].is_move && !shapes[i].is_input) shapes[i].AddAcceleration({ 0.0f, g });

            if (std::find(shapeID.begin(), shapeID.end(), i) != shapeID.end()) continue;

            // Shapes in bounds
            if (!shapes[i].IsConstrained({ 0.0f, 0.0f }, { screen_size.x, screen_size.y })) { shapes.erase(shapes.begin() + i); }
        }

        std::vector<Manifold> manifolds;
        for (uint32_t a = 0; a < shapes.size() - 1; a++) {
            for (uint32_t b = a + 1; b < shapes.size(); b++) {
                const auto& manifold_data = RigidBody::SATOverlap(shapes[a], shapes[b]);
                if (manifold_data.first > 0.0f) {
                    // Add to vector if overlap > 0
                    manifolds.push_back(Manifold(&shapes[a], &shapes[b], manifold_data.first, manifold_data.second));
                    shapes[a].is_collide = true;
                    shapes[b].is_collide = true;
                }
            }
        }

        for (auto& m : manifolds) { m.Logic(inv_FPS, 5, is_debug); }
        for (auto& s : shapes) {
            s.Logic(inv_FPS, 0.0f, is_debug);
            
            // If polygon is not in contact, then the polygon should be moving by gravity
            if (!s.is_collide) s.is_move = true;
            s.is_collide = false;
        }
    }

    alpha = accumulator / inv_FPS;
}

void Scene::Draw(olc::PixelGameEngine* pge, bool is_fill) {
    for (auto& s : shapes) s.Draw(pge, is_fill, 0.0f);
    for (auto& c : constraints) c.Draw(pge);
}

void Scene::DrawDecals(olc::PixelGameEngine* pge) {
    for (auto& s : shapes) s.Draw(pge, 0.0f);
    for (auto& c : constraints) c.Draw(pge);
}

void Scene::AddShape(const olc::vf2d& p, int n_vertices, float len, float angle, float angular_velocity, float mass, olc::Pixel color, float e, float sf, float df) {
    RigidBody rb(p, n_vertices, len, angle, angular_velocity, mass, e, sf, df, shapes.size());
    rb.SetColor(color);
    shapes.push_back(rb);
}

void Scene::AddConstraint(const olc::vf2d& p, float len, float k, float b, int n, int index) {
    Constraint c(p, len, k, b, n);
    if (index >= 0) c.Attach(index);
    constraints.push_back(c);
}