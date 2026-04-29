#include <bits/stdc++.h>
using namespace std;

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double norm() const { return sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double n = norm(); return {x/n, y/n, z/n}; }
};

struct Triangle {
    Vec3 v[3];
    Vec3 normal() const { return (v[1]-v[0]).cross(v[2]-v[0]).normalized(); }
    Vec3 centroid() const { return (v[0]+v[1]+v[2]) * (1.0/3.0); }
};

struct AABB {
    Vec3 mn, mx;
    AABB(const Triangle& t) {
        mn = mx = t.v[0];
        for (int i = 1; i < 3; i++) {
            mn.x = min(mn.x, t.v[i].x); mn.y = min(mn.y, t.v[i].y); mn.z = min(mn.z, t.v[i].z);
            mx.x = max(mx.x, t.v[i].x); mx.y = max(mx.y, t.v[i].y); mx.z = max(mx.z, t.v[i].z);
        }
    }
    bool intersects(const AABB& o) const {
        return mn.x<=o.mx.x && mx.x>=o.mn.x &&
               mn.y<=o.mx.y && mx.y>=o.mn.y &&
               mn.z<=o.mx.z && mx.z>=o.mn.z;
    }
};

// Ray-triangle intersection (Möller–Trumbore)
bool rayTriangleIntersect(const Vec3& orig, const Vec3& dir, const Triangle& tri, double& t) {
    const double EPS = 1e-9;
    Vec3 e1 = tri.v[1]-tri.v[0], e2 = tri.v[2]-tri.v[0];
    Vec3 h = dir.cross(e2);
    double a = e1.dot(h);
    if (fabs(a) < EPS) return false;
    double f = 1.0/a;
    Vec3 s = orig - tri.v[0];
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;
    Vec3 q = s.cross(e1);
    double v = f * dir.dot(q);
    if (v < 0.0 || u+v > 1.0) return false;
    t = f * e2.dot(q);
    return t > EPS;
}

// Point inside mesh test using ray casting
bool pointInsideMesh(const Vec3& p, const vector<Triangle>& mesh) {
    Vec3 dir = {1.0, 0.0, 0.0};
    int count = 0;
    for (const auto& tri : mesh) {
        double t;
        if (rayTriangleIntersect(p, dir, tri, t))
            count++;
    }
    return count % 2 == 1;
}

enum BoolOp { UNION, INTERSECTION, DIFFERENCE };

vector<Triangle> booleanOp(const vector<Triangle>& A, const vector<Triangle>& B, BoolOp op) {
    vector<Triangle> result;
    for (const auto& tri : A) {
        Vec3 c = tri.centroid();
        bool inside = pointInsideMesh(c, B);
        if ((op == UNION && !inside) ||
            (op == INTERSECTION && inside) ||
            (op == DIFFERENCE && !inside))
            result.push_back(tri);
    }
    for (const auto& tri : B) {
        Vec3 c = tri.centroid();
        bool inside = pointInsideMesh(c, A);
        if ((op == UNION && !inside) ||
            (op == INTERSECTION && inside))
            result.push_back(tri);
        else if (op == DIFFERENCE && inside) {
            Triangle flipped = tri;
            swap(flipped.v[1], flipped.v[2]);
            result.push_back(flipped);
        }
    }
    return result;
}

vector<Triangle> makeCube(Vec3 offset, double s) {
    vector<Vec3> v = {
        {0,0,0},{s,0,0},{s,s,0},{0,s,0},
        {0,0,s},{s,0,s},{s,s,s},{0,s,s}
    };
    for (auto& p : v) p = p + offset;
    vector<array<int,3>> faces = {
        {0,2,1},{0,3,2},{4,5,6},{4,6,7},
        {0,1,5},{0,5,4},{1,2,6},{1,6,5},
        {2,3,7},{2,7,6},{3,0,4},{3,4,7}
    };
    vector<Triangle> mesh;
    for (auto& f : faces)
        mesh.push_back({v[f[0]], v[f[1]], v[f[2]]});
    return mesh;
}

int main() {
    auto cubeA = makeCube({0,0,0}, 2.0);
    auto cubeB = makeCube({1,1,1}, 2.0);

    auto uni  = booleanOp(cubeA, cubeB, UNION);
    auto inter = booleanOp(cubeA, cubeB, INTERSECTION);
    auto diff = booleanOp(cubeA, cubeB, DIFFERENCE);

    cout << "Union triangles:        " << uni.size()   << "\n";
    cout << "Intersection triangles: " << inter.size() << "\n";
    cout << "Difference triangles:   " << diff.size()  << "\n";
    return 0;
}