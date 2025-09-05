#pragma once

#include <vector>
#include <string>
#include <format>
#include <sstream>
#include <iomanip>
#include <PxPhysicsAPI.h>

inline std::string to_string(physx::PxVec3 point) {
    return std::format("({:.7f}, {:.7f}, {:.7f})", point.x, point.y, point.z);
}
inline std::ostream& operator<<(std::ostream& os, const physx::PxVec3& v) {
    os << to_string(v);
    return os;
}

namespace phylib {
    using namespace physx;
    // forward declarations
    class Obj;
    struct ProximityResult;
    struct ConvexSupport;

    // API functions

    // load an 'obj' file
    Obj parseObjFile(const std::string& path);

    // create a dynamic actor from an Obj
    PxRigidDynamic* createActor(const Obj& obj, const PxTransform& pose, PxPhysics& physics, const PxMaterial& material);

    // GJK API call with 2 dynamics
    ProximityResult proximityBetweenDynamics(PxRigidDynamic* a, PxRigidDynamic* b);

    // dump (the cooked) actor to an 'obj' file
    std::string dumpActorToObjString(PxRigidDynamic* actor);
    void dumpActorToObjFile(PxRigidDynamic* actor, const std::string& filename);


    // data structures
    struct Obj {
        struct Shape {
            std::vector<PxVec3> vertices;
            std::vector<PxU32> indices; // every 3 indices form a triangle
            std::vector<PxHullPolygon> polygons; // indices into vertices
        };
        std::vector<Shape> shapes;
        std::string to_string() {
            std::stringstream ss;
            for (size_t i = 0; i < shapes.size(); i++) {
                ss << "obj_" << i << "\n";
                for (const auto& v : shapes[i].vertices) {
                    ss << std::format("  v {:.6f} {:.6f} {:.6f}\n", v.x, v.y, v.z);
                }
                for (uint32_t j = 0; j < shapes[i].indices.size() -2; j+=3) {
                    // every 3 indices form a triangle
                    ss << std::format("  f {} {} {}\n", shapes[i].indices[j], shapes[i].indices[j+1], shapes[i].indices[j+2]);
                }
            }
            return ss.str();
        }
    };

    struct ProximityResult {
        bool found{false};       // true if at least one convex-convex pair was evaluated
        PxVec3 pointA{0,0,0};    // nearest point on actor A
        PxVec3 pointB{0,0,0};    // nearest point on actor B
        PxVec3 axis{0,0,0};      // direction from pointA to pointB
        PxReal separation{PX_MAX_F32}; // distance between pointA and pointB
        std::string to_string() {
            return std::format(
                "found = {} point A = {}, point B = {}, separation = {}\n",
                found, ::to_string(pointA), ::to_string(pointB), separation);
        }
    };

    struct ConvexSupport : public PxGjkQuery::Support {
        const PxConvexMesh* convex;
        explicit ConvexSupport(const PxConvexMesh* c) : convex(c) {}
        PxReal getMargin() const override { return 0.0f; }
        PxVec3 supportLocal(const PxVec3& dir) const override {
            PxU32 n = convex->getNbVertices();
            const PxVec3* verts = convex->getVertices();
            PxReal best = -PX_MAX_F32;
            PxVec3 bestV(0);
            for (PxU32 i = 0; i < n; i++) {
                PxReal proj = verts[i].dot(dir);
                if (proj > best) {
                    best = proj;
                    bestV = verts[i];
                }
            }
            return bestV;
        }
    };
}
