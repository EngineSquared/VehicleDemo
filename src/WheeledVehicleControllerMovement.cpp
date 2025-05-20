#include "WheeledVehicleControllerMovement.hpp"

#include "WheeledVehicle3D.hpp"
#include "Logger.hpp"
#include "JoltPhysics.hpp"

constexpr int JOYSTICK_ID = 0;
constexpr int PS5_L2_AXIS = 2;
constexpr int PS5_R2_AXIS = 5;
constexpr int PS5_CIRCLE_BUTTON = 1;

// Tested on PS5 controller
void WheeledVehicleControllerMovement::operator()(ES::Engine::Core &core) const
{
    

    if (!ES::Plugin::Input::Utils::IsJoystickPresent(JOYSTICK_ID))
    {
        return;
    }

    if (!entity.template HasComponents<ES::Plugin::Physics::Component::WheeledVehicle3D, ES::Plugin::Physics::Component::RigidBody3D>(core))
    {
        ES::Utils::Log::Error(fmt::format("WheeledVehicleControllerMovement component is not fully initialized for entity {}",
                                          static_cast<uint32_t>(entity)));
        return;
    }
    auto &physicsSystem = core.GetResource<ES::Plugin::Physics::Resource::PhysicsManager>().GetPhysicsSystem();

    auto &wheeledVehicle = entity.template GetComponents<ES::Plugin::Physics::Component::WheeledVehicle3D>(core);
    auto &rigidBody = entity.template GetComponents<ES::Plugin::Physics::Component::RigidBody3D>(core);

    auto joystickAxes = ES::Plugin::Input::Utils::GetJoystickAxes(JOYSTICK_ID);
    auto joystickButtons = ES::Plugin::Input::Utils::GetJoystickButtons(JOYSTICK_ID);

    if (joystickAxes.size() < 4)
    {
        return;
    }

    for (auto i = 0; i < joystickButtons.size(); ++i)
    {
        printf("Button %d: %d\n", i, joystickButtons[i]);
    }

    for (auto i = 0; i < joystickAxes.size(); ++i)
    {
        printf("Axis %d: %f\n", i, joystickAxes[i]);
    }

    auto vehicleSpeedJph = rigidBody.body->GetLinearVelocity();
    auto vehicleSpeed = glm::vec3(vehicleSpeedJph.GetX(), 0.0f, vehicleSpeedJph.GetZ());
    auto vehicleSpeedMagnitude = glm::length(vehicleSpeed);
    bool shouldReverse = false;
    if (vehicleSpeedMagnitude <= 0.1f)
    {
        shouldReverse = true;
    }

    // L2 is throttle and R2 is brake
    // Automatic gearbox : if the car speed is very slow, the brake key is used as reverse, like in most games
    auto throttle = joystickAxes[1];
    
    

    
    

    // auto throttle = forwardForce + reverseForce;
    // auto steering = leftForce + rightForce;

    // wheeledVehicle.SetDriverInput(throttle, steering, brakeForce, handbrakeForce);

    // if (throttle != 0.0f || steering != 0.0f || brakeForce != 0.0f || handbrakeForce != 0.0f)
    // {
    //     
    //     physicsSystem.GetBodyInterface().ActivateBody(rigidBody.body->GetID());
    // }
}