#include "CreateVehicle.hpp"

#include "CreateBox.hpp"
#include "CreateCylinder.hpp"
#include "VehicleControllerRef.hpp" // TODO: remove this when encapsulated
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

static ES::Engine::Entity CreateVehicleWheel(
    ES::Engine::Core &core,
    const glm::vec3 &position,
    float radius,
    float width
)
{
    // Rotate the wheel 90 degrees around the Z axis so it's oriented correctly
    glm::quat rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1, 0, 0));
    ES::Engine::Entity wheel = CreateCylinder(
        core,
        position,
        rotation,
        glm::vec3(radius, width, radius)
    );

    wheel.AddComponent<ES::Plugin::OpenGL::Component::ShaderHandle>(core, "no_light");
    wheel.AddComponent<ES::Plugin::OpenGL::Component::MaterialHandle>(core, "car_wheel");
    wheel.AddComponent<ES::Plugin::OpenGL::Component::ModelHandle>(core, "car_wheel");

    return wheel;
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

    std::array<ES::Engine::Entity, 4> wheels;

    for (int i = 0; i < 4; ++i) {
        glm::vec3 wheelPosition = bodyPosition;
        wheelPosition.x += (i % 2 == 0 ? -1 : 1) * wheelOffsetHorizontal;
        wheelPosition.z += (i / 2 == 0 ? -1 : 1) * wheelOffsetHorizontal;
        wheels[i] = CreateVehicleWheel(core, wheelPosition, wheelRadius, wheelWidth);
    }

    // Create the vehicle body
    ES::Engine::Entity vehicleBody = CreateVehicleBody(
        core,
        bodyPosition,
        halfVehicleLength,
        halfVehicleWidth,
        halfVehicleHeight
    );
    
    // Create the vehicle constraint
    // TODO: this will need to be encapsulated
    static auto vehicleConstraintSettings = std::make_shared<JPH::VehicleConstraintSettings>();
    vehicleConstraintSettings->mMaxPitchRollAngle = 1.0f; // ~60 degrees
    vehicleConstraintSettings->mWheels.resize(4);

    // Set up the wheels
    static auto fl = std::make_shared<JPH::WheelSettingsWV>();
    fl->mRadius = wheelRadius;
    fl->mWidth = wheelWidth;
    fl->mSuspensionMinLength = suspensionMinLength;
    fl->mSuspensionMaxLength = suspensionMaxLength;
    fl->mMaxSteerAngle = maxSteerAngle;
    fl->mMaxHandBrakeTorque = 0.0f; // Front wheels don't have handbrake
    fl->SetEmbedded();
    vehicleConstraintSettings->mWheels[0] = fl.get();

    static auto fr = std::make_shared<JPH::WheelSettingsWV>();
    fr->mRadius = wheelRadius;
    fr->mWidth = wheelWidth;
    fr->mSuspensionMinLength = suspensionMinLength;
    fr->mSuspensionMaxLength = suspensionMaxLength;
    fr->mMaxSteerAngle = maxSteerAngle;
    fr->mMaxHandBrakeTorque = 0.0f; // Front wheels don't have handbrake
    fr->SetEmbedded();
    vehicleConstraintSettings->mWheels[1] = fr.get();

    static auto rl = std::make_shared<JPH::WheelSettingsWV>();
    rl->mRadius = wheelRadius;
    rl->mWidth = wheelWidth;
    rl->mSuspensionMinLength = suspensionMinLength;
    rl->mSuspensionMaxLength = suspensionMaxLength;
    rl->mMaxSteerAngle = 0.0f; // Rear wheels don't have steering
    rl->SetEmbedded();
    vehicleConstraintSettings->mWheels[2] = rl.get();

    static auto rr = std::make_shared<JPH::WheelSettingsWV>();
    rr->mRadius = wheelRadius;
    rr->mWidth = wheelWidth;
    rr->mSuspensionMinLength = suspensionMinLength;
    rr->mSuspensionMaxLength = suspensionMaxLength;
    rr->mMaxSteerAngle = 0.0f; // Rear wheels don't have steering
    rr->SetEmbedded();
    vehicleConstraintSettings->mWheels[3] = rr.get();

    // Set up the vehicle controller
    static auto vehicleControllerSettings = std::make_shared<JPH::WheeledVehicleControllerSettings>();
    vehicleControllerSettings->SetEmbedded();
    vehicleControllerSettings->mEngine.mMaxTorque = maxEngineTorque;
    vehicleControllerSettings->mTransmission.mClutchStrength = clutchStrength;

    // Set the controller for the constraint
    vehicleConstraintSettings->mController = vehicleControllerSettings.get();

    // Differentials
    if (fourWheelDrive) {
        vehicleControllerSettings->mDifferentials.resize(2);
    } else {
        vehicleControllerSettings->mDifferentials.resize(1);
    }

    // Front differential
    auto frontDifferentialSettings = JPH::VehicleDifferentialSettings();
    frontDifferentialSettings.mLeftWheel = 0;
    frontDifferentialSettings.mRightWheel = 1;
    frontDifferentialSettings.mLimitedSlipRatio = leftRightLimitedSlipRatio;
    if (fourWheelDrive) {
        frontDifferentialSettings.mEngineTorqueRatio = 0.5f;
    }
    vehicleControllerSettings->mDifferentials[0] = frontDifferentialSettings;
    vehicleControllerSettings->mDifferentialLimitedSlipRatio = frontBackLimitedSlipRatio;

    // Rear differential
    if (fourWheelDrive) {
        auto rearDifferentialSettings = JPH::VehicleDifferentialSettings();
        rearDifferentialSettings.mLeftWheel = 2;
        rearDifferentialSettings.mRightWheel = 3;
        rearDifferentialSettings.mLimitedSlipRatio = leftRightLimitedSlipRatio;
        rearDifferentialSettings.mEngineTorqueRatio = 0.5f;
        vehicleControllerSettings->mDifferentials[1] = rearDifferentialSettings;
    }

    // Anti-roll bars
    vehicleConstraintSettings->mAntiRollBars.resize(2);
    if (antiRollBar) {
        auto frontAntiRollBarSettings = JPH::VehicleAntiRollBar();
        frontAntiRollBarSettings.mLeftWheel = 0;
        frontAntiRollBarSettings.mRightWheel = 1;
        vehicleConstraintSettings->mAntiRollBars[0] = frontAntiRollBarSettings;

        auto rearAntiRollBarSettings = JPH::VehicleAntiRollBar();
        rearAntiRollBarSettings.mLeftWheel = 2;
        rearAntiRollBarSettings.mRightWheel = 3;
        vehicleConstraintSettings->mAntiRollBars[1] = rearAntiRollBarSettings;
    }

    auto carJoltBody = vehicleBody.GetComponents<ES::Plugin::Physics::Component::RigidBody3D>(core).body;
    static auto vehicleConstraint = std::make_shared<JPH::VehicleConstraint>(*carJoltBody, *vehicleConstraintSettings.get());
    vehicleConstraint->SetEmbedded();
    vehicleConstraint->SetVehicleCollisionTester(new JPH::VehicleCollisionTesterCastCylinder(
        ES::Plugin::Physics::Utils::Layers::MOVING,
        0.05
    ));

    VehicleControllerRef vehicleControllerRef(vehicleConstraint->GetController(), carJoltBody->GetID());
    core.RegisterResource<VehicleControllerRef>(std::move(vehicleControllerRef));

    physicsSystem.AddConstraint(vehicleConstraint.get());
    physicsSystem.AddStepListener(vehicleConstraint.get());
}