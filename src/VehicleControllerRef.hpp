#pragma once

// TODO: replace this with proper encapsulation, this is a hack

#include "JoltPhysics.hpp"

#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

struct VehicleControllerRef {
    JPH::VehicleController *controller = nullptr;
    JPH::BodyID bodyId;
};
