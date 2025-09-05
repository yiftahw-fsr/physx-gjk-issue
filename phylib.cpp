#include "phylib.hpp"

#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ranges>
#include <cassert>

#include <PxPhysicsAPI.h>
#include <geometry/PxGeometryQuery.h>

#include "phylib.hpp"

#include <iomanip>

namespace phylib {
    using namespace physx;

    // Helper: split OBJ into multiple convex vertex sets
    Obj parseObjFile(const std::string& path) {
        Obj result;
        Obj::Shape currentShape;

        std::ifstream file(path);

        if (!file.is_open()) {
            std::cerr << "Failed to open OBJ file: " << path << "\n";
            return result;
        }

        std::string line;
        uint32_t total_vertices_found = 0;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#')
                continue;

            if (line.rfind("o ", 0) == 0) {
                // Start of new object: flush previous
                if (!currentShape.vertices.empty()) {
                    total_vertices_found += static_cast<uint32_t>(currentShape.vertices.size());
                    result.shapes.push_back(currentShape);
                    currentShape = Obj::Shape();
                }
            }
            else if (line.rfind("v ", 0) == 0) {
                std::istringstream ls(line.substr(2));
                float x, y, z;
                if (ls >> x >> y >> z) {
                    currentShape.vertices.push_back(PxVec3(x, y, z));
                }
            }
            else if (line.rfind("f ", 0) == 0) {
                std::istringstream ls(line.substr(2));
                std::string vertex;
                while (ls >> vertex) {
                    // Face definitions can be "f v", "f v/vt", "f v/vt/vn", or "f v//vn"
                    size_t slash1 = vertex.find('/');
                    PxU32 vIdx = 0;
                    try {
                        if (slash1 == std::string::npos) {
                            vIdx = std::stoi(vertex);
                        } else {
                            vIdx = std::stoi(vertex.substr(0, slash1));
                        }
                        vIdx -= total_vertices_found + 1; // Adjust for previous shapes and convert to 0-based
                        if (vIdx >= currentShape.vertices.size()) {
                            std::cerr << "Warning: Face index out of bounds in OBJ file: " << vIdx << ", num vertices: "<< currentShape.vertices.size() << " line: " << line << "\n";
                        }
                        currentShape.indices.push_back(vIdx); // OBJ is 1-indexed
                    } catch (...) {
                        std::cerr << "Warning: Invalid face index in OBJ file: " << vertex << "\n";
                    }
                }
            }
        }

        // Push last object if not empty
        if (!currentShape.vertices.empty()) {
            result.shapes.push_back(currentShape);
        }

        // fill the polygons data structure
        // 1. compute the plane equation
        for (auto& shape : result.shapes) {
            size_t numPolygons = shape.indices.size() / 3;
            shape.polygons.resize(numPolygons);
            for (size_t i = 0; i < numPolygons; i++) {
                PxVec3 v0 = shape.vertices[shape.indices[i * 3 + 0]];
                PxVec3 v1 = shape.vertices[shape.indices[i * 3 + 1]];
                PxVec3 v2 = shape.vertices[shape.indices[i * 3 + 2]];
                PxVec3 normal = (v1 - v0).cross(v2 - v0);
                normal.normalize();
                float d = -normal.dot(v0);
                shape.polygons[i].mNbVerts = 3;
                shape.polygons[i].mIndexBase = PxU16(i * 3);
                shape.polygons[i].mPlane[0] = normal.x;
                shape.polygons[i].mPlane[1] = normal.y;
                shape.polygons[i].mPlane[2] = normal.z;
                shape.polygons[i].mPlane[3] = d;
            }
        }
        return result;
    }

    PxConvexMesh* cookConvexMesh(const Obj::Shape& shape,
                                 PxPhysics& physics, bool use_original_polygons = true)
    {
        PxConvexMeshDesc desc;
        desc.points.count  = PxU32(shape.vertices.size());
        desc.points.stride = sizeof(PxVec3);
        desc.points.data   = shape.vertices.data();

        if (use_original_polygons) {
            desc.polygons.count  = PxU32(shape.polygons.size());
            desc.polygons.stride = sizeof(PxHullPolygon);
            desc.polygons.data   = shape.polygons.data();

            desc.indices.count  = PxU32(shape.indices.size());
            desc.indices.stride = sizeof(PxU32);
            desc.indices.data   = shape.indices.data();
            desc.flags         = PxConvexFlags(0);
        } else {
            desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
        }

        PxDefaultMemoryOutputStream out;
        PxConvexMeshCookingResult::Enum result;
        bool status = PxCookConvexMesh(PxCookingParams(PxTolerancesScale()), desc, out, &result);
        if (!status) {
            std::cerr << "Convex cooking failed!\n";
            return nullptr;
        }

        PxDefaultMemoryInputData in(out.getData(), out.getSize());
        return physics.createConvexMesh(in);
    }

    std::vector<PxConvexMesh*> createConvexGroup(const Obj& obj, PxPhysics& physics) {
        std::vector<PxConvexMesh*> convexGroup;
        for (const auto& shape : obj.shapes) {
            auto result = cookConvexMesh(shape, physics, false);
            if (!result) {
                // Cleanup previously created meshes
                std::cerr << "Convex cooking failed!\n";
                for (auto* mesh : convexGroup) {
                    mesh->release();
                }
                convexGroup.clear();
                return {};
            }
            convexGroup.push_back(result);
        }
        return convexGroup;
    }

    PxRigidDynamic* createRigidDynamicFromConvexGroup(std::vector<PxConvexMesh*> convexes, const PxTransform& pose, PxPhysics& physics, const PxMaterial& material)
    {
        // Create the dynamic actor at the given world pose
        PxRigidDynamic* actor = physics.createRigidDynamic(pose);

        for (auto* convex : convexes) {
            if (!convex) continue;

            // Wrap the convex mesh in a geometry
            PxConvexMeshGeometry geom(convex);

            // Create a shape and attach it to the actor
            PxShape* shape = physics.createShape(geom, material);
            actor->attachShape(*shape);

            // Release the shape reference (actor keeps its own)
            shape->release();
        }

        return actor;
    }

    PxRigidDynamic* createActor(const Obj& obj, const PxTransform& pose, PxPhysics& physics, const PxMaterial& material)
    {
        const auto convexes = createConvexGroup(obj, physics);
        if (convexes.empty()) {
            return nullptr;
        }
        return createRigidDynamicFromConvexGroup(convexes, pose, physics, material);
    }

    std::vector<std::pair<const PxConvexMesh*, PxTransform>> getConvexParts(PxRigidDynamic* actor)
    {
        std::vector<std::pair<const PxConvexMesh*, PxTransform>> parts;

        PxU32 nbShapes = actor->getNbShapes();
        std::vector<PxShape*> shapes(nbShapes);
        actor->getShapes(shapes.data(), nbShapes);

        for (PxShape* shape : shapes) {
            PxGeometryHolder geom = shape->getGeometry();
            if (geom.getType() == PxGeometryType::eCONVEXMESH) {
                const PxConvexMesh* cm = geom.convexMesh().convexMesh;

                // World pose = actor pose * shape local pose
                PxTransform globalPose = actor->getGlobalPose() * shape->getLocalPose();

                parts.emplace_back(cm, globalPose);
            }
        }
        return parts;
    }

    ProximityResult proximityBetweenDynamics(PxRigidDynamic* a, PxRigidDynamic* b)
    {
        auto partsA = getConvexParts(a);
        auto partsB = getConvexParts(b);

        PxReal contactDistance = 500.0f;
        PxReal toleranceLength = 1e-3f;

        ProximityResult best;
        for (auto& [cmA, poseA] : partsA) {
            ConvexSupport supA(cmA);

            for (auto& [cmB, poseB] : partsB) {
                ConvexSupport supB(cmB);

                PxVec3 pA, pB, axis;
                PxReal sep = 0.0f;

                PxGjkQuery::proximityInfo(
                    supA, supB,
                    poseA, poseB,
                    contactDistance, toleranceLength,
                    pA, pB, axis, sep
                );

                // Always track the minimum distance, regardless of "contact"
                if (!best.found || sep < best.separation) {
                    best.found = true;
                    best.pointA = pA;
                    best.pointB = pB;
                    best.separation = sep;
                    best.axis = axis;
                }
            }
        }
        return best;
    }

    std::string dumpConvexMeshToObjString(PxConvexMesh* convex, const PxTransform& pose, uint32_t num_vertices_dumped)
    {
        std::ostringstream out;

        // vertices
        const PxVec3* verts = convex->getVertices();
        PxU32 nbVerts = convex->getNbVertices();
        for (PxU32 i = 0; i < nbVerts; ++i) {
            PxVec3 vWorld = pose.transform(verts[i]); // transform to world if desired
            out << std::setprecision(6);
            out << "v " << vWorld.x << " " << vWorld.y << " " << vWorld.z << "\n";
        }

        // faces
        PxHullPolygon poly;
        const auto* indices = convex->getIndexBuffer();
        PxU32 nbPolys = convex->getNbPolygons();
        for (PxU32 i = 0; i < nbPolys; ++i) {
            convex->getPolygonData(i, poly);
            out << "f";
            for (PxU32 j = 0; j < poly.mNbVerts; ++j) {
                PxU32 idx = indices[poly.mIndexBase + j] + num_vertices_dumped;
                out << " " << (idx + 1); // OBJ is 1-indexed
            }
            out << "\n";
        }

        return out.str();
    }

    std::string dumpActorToObjString(PxRigidDynamic* actor) {
        std::ostringstream out;
        uint32_t num_vertices_dumped = 0;
        for (const auto &[index, pair] : std::views::enumerate(getConvexParts(actor))) {
            const auto &[mesh, pose] = pair;
            out << "o convex_" << index << "\n";
            out << dumpConvexMeshToObjString(const_cast<PxConvexMesh*>(mesh), pose, num_vertices_dumped) << "\n";
            num_vertices_dumped += mesh->getNbVertices();
        }
        return out.str();
    }

    void dumpActorToObjFile(PxRigidDynamic* actor, const std::string& filename) {
        auto content = dumpActorToObjString(actor);
        std::ofstream file(filename);
        if (file.is_open()) {
            file << content;
            file.close();
        } else {
            std::cerr << "Failed to open file for writing: " << filename << "\n";
        }
    }
}
