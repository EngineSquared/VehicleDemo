#include "VehicleMovement.hpp"

#include "VehicleControllerRef.hpp"

#include "Input.hpp"

void VehicleMovement(ES::Engine::Core &core)
{
    // TODO: this is a hack, should be encapsulated properly in ESQ
    auto &controllerRef = core.GetResource<VehicleControllerRef>();
    // auto controller = dynamic_cast<JPH::WheeledVehicleController *>(controllerRef.controller);

    // if (controller == nullptr) {
    //     // Should never happen
    //     throw std::runtime_error("VehicleControllerRef is not a WheeledVehicleController");
    // }

    // auto throttle = ES::Plugin::Input::Utils::IsKeyPressed(GLFW_KEY_W) ? 1.0f : 0.0f;
    // auto left = ES::Plugin::Input::Utils::IsKeyPressed(GLFW_KEY_A) ? 1.0f : 0.0f;
    // auto right = ES::Plugin::Input::Utils::IsKeyPressed(GLFW_KEY_D) ? 1.0f : 0.0f;
    // auto steer = right - left;
    // auto brake = ES::Plugin::Input::Utils::IsKeyPressed(GLFW_KEY_S) ? 1.0f : 0.0f;
    // throttle = brake == 1.0f ? 0.0f : throttle;


    // controller->SetDriverInput(throttle, steer, brake, 0.0f);
    // // TODO: not always activate
    // auto &physicsSystem = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>().GetPhysicsSystem();
    // physicsSystem.GetBodyInterface().ActivateBody(controllerRef.bodyId);
    // auto pos = physicsSystem.GetBodyInterface().GetPosition(controllerRef.bodyId);
    // printf("Vehicle position: %f %f %f\n", pos.GetX(), pos.GetY(), pos.GetZ());
}