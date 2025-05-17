#include "CreateVehicle.hpp"

#include "CreateBox.hpp"
#include "JoltPhysics.hpp"
#include "OpenGL.hpp"

#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/VehicleEngine.h>
#include <Jolt/Physics/Vehicle/VehicleTrack.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/Physics/Vehicle/Wheel.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

static ES::Engine::Entity CreateVehicleBody(
    ES::Engine::Core &core,
    const glm::vec3 &position,
    float halfVehicleLength,
    float halfVehicleWidth,
    float halfVehicleHeight
)
{
    std::shared_ptr<JPH::ShapeSettings> carShapeSettingsBox = std::make_shared<JPH::BoxShapeSettings>(
        JPH::Vec3(halfVehicleLength, halfVehicleHeight, halfVehicleWidth)
    );
    carShapeSettingsBox->SetEmbedded();
    std::shared_ptr<JPH::ShapeSettings> carShapeSettings = std::make_shared<JPH::OffsetCenterOfMassShapeSettings>(
        JPH::Vec3(0.0f, -halfVehicleHeight, 0.0f),
        carShapeSettingsBox.get()
    );
	carShapeSettings->SetEmbedded();

    ES::Engine::Entity vehicleBody = core.CreateEntity();
    vehicleBody.AddComponent<ES::Plugin::Object::Component::Transform>(core, position);
    vehicleBody.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(
        core,
        carShapeSettings,
        JPH::EMotionType::Dynamic,
        ES::Plugin::Physics::Utils::Layers::MOVING);

    vehicleBody.AddComponent<ES::Plugin::OpenGL::Component::ShaderHandle>(core, "no_light");
    vehicleBody.AddComponent<ES::Plugin::OpenGL::Component::MaterialHandle>(core, "car_body");
    vehicleBody.AddComponent<ES::Plugin::OpenGL::Component::ModelHandle>(core, "car_body");
    vehicleBody.AddComponent<ES::Plugin::Object::Component::Mesh>(core, CreateBoxMesh(
        glm::vec3(halfVehicleLength, halfVehicleHeight, halfVehicleWidth)
    ));

    return vehicleBody;

}

void CreateVehicle(ES::Engine::Core &core)
{
    // consts
    glm::vec3 bodyPosition(0.0f, 6.0f, 0.0f);

    float wheelRadius = 0.3f;
    float wheelWidth = 0.1f;
    float halfVehicleLength = 2.0f;
    float halfVehicleWidth = 0.9f;
    float halfVehicleHeight = 0.2f;
    float wheelOffsetHorizontal = 1.4f;
    float wheelOffsetVertical = 0.18f;
    float suspensionMinLength = 0.3f;
    float suspensionMaxLength = 0.5f;
    float maxSteerAngle = 0.52f; // in radians, ~30 degrees
    bool fourWheelDrive = true;
    float frontBackLimitedSlipRatio = 1.4f;
    float leftRightLimitedSlipRatio = 1.4f;
    bool antiRollBar = true;

    float vehicleMass = 1500.0f;
    float maxEngineTorque = 500.0f;
    float clutchStrength = 10.0f;

    // PhysicsManager
    auto &physicsManager = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>();
    auto &physicsSystem = physicsManager.GetPhysicsSystem();

    // Create the vehicle body
    ES::Engine::Entity vehicleBody = CreateVehicleBody(
        core,
        bodyPosition,
        halfVehicleLength,
        halfVehicleWidth,
        halfVehicleHeight
    );
}