// main.cpp
#include <PxPhysicsAPI.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <ranges>
#include <map>
#include <cassert>

#include "phylib.hpp"

using namespace physx;

int main() {
    // PhysX initialization
    auto allocator = PxDefaultAllocator();
    auto error_callback = PxDefaultErrorCallback();
    auto foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, error_callback);
    auto physx = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true, nullptr);
    if (!physx) {
        std::cerr << "PxCreatePhysics failed!\n";
        return 1;
    }
    PxMaterial *material = physx->createMaterial(0.0f, 5.0f, 0.0f);

    // Load actors from OBJ files and hard coded poses
    constexpr std::string_view p3l0_name = "p3l_0";
    constexpr std::string_view p3l1_name = "p3l_1";
    constexpr std::string_view p3r_name = "p3r";

    constexpr std::string_view p3l_filename = "p3l.obj";
    constexpr std::string_view p3r_filename = "p3r.obj";

    std::array<PxTransform, 2> p3l_poses {
        PxTransform{PxVec3{-382.41943359,-36.63606644,538.67352295}, PxQuat{0.90120637,-0.04327928,-0.26553822,-0.33976975}},
        PxTransform{PxVec3{-382.51943359,-36.63606644,538.67352295}, PxQuat{0.90120637,-0.04327928,-0.26553822,-0.33976975}}
    };

    PxTransform p3r_pose {PxVec3{-78.02865601,328.71896362,584.83435059}, PxQuat{0.70096153,-0.47570169,-0.52838022,0.05634902}};

    std::map<std::string_view, PxRigidDynamic*> actorMap;
    auto createActor = [&](const std::string_view& name, const std::string_view& filename, const PxTransform& pose) {
        const auto obj = phylib::parseObjFile(std::string(filename));
        const auto actor = phylib::createActor(obj, pose, *physx, *material);
        assert(actor);
        actorMap[name] = actor;
        phylib::dumpActorToObjFile(actor, std::format("{}_cooked.obj", name));
        std::cout << "Loaded Obj from " << filename << "\n";
    };

    createActor(p3l0_name, p3l_filename, p3l_poses[0]);
    createActor(p3l1_name, p3l_filename, p3l_poses[1]);
    createActor(p3r_name, p3r_filename, p3r_pose);

    auto result0 = phylib::proximityBetweenDynamics(actorMap[p3l0_name], actorMap[p3r_name]);
    std::cout << "Proximity between " << p3l0_name << " and " << p3r_name << ":\n";
    std::cout << result0.to_string() << "\n";

    auto result1 = phylib::proximityBetweenDynamics(actorMap[p3l1_name], actorMap[p3r_name]);
    std::cout << "Proximity between " << p3l1_name << " and " << p3r_name << ":\n";
    std::cout << result1.to_string() << "\n";



    return 0;
}
