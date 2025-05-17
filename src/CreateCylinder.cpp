#include "CreateCylinder.hpp"

#include "JoltPhysics.hpp"
#include "OpenGL.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <Jolt/Physics/Collision/Shape/CylinderShape.h>

ES::Plugin::Object::Component::Mesh CreateCylinderMesh(
	const glm::vec3 &size,
	int segments)
{
	ES::Plugin::Object::Component::Mesh mesh;

	float radius = size.x;
	float height = size.y;

	// Top and bottom center points
	glm::vec3 top_center(0.0f, height, 0.0f);
	glm::vec3 bottom_center(0.0f, -height, 0.0f);

	// Generate side vertices
	for (int i = 0; i < segments; ++i) {
		float angle = 2.0f * glm::pi<float>() * i / segments;
		float x = radius * cos(angle);
		float z = radius * sin(angle);

		glm::vec3 top(x, height, z);
		glm::vec3 bottom(x, -height, z);
		glm::vec3 normal = glm::normalize(glm::vec3(x, 0.0f, z));

		// Side quad (2 vertices per side)
		mesh.vertices.push_back(top);
		mesh.normals.push_back(normal);
		mesh.vertices.push_back(bottom);
		mesh.normals.push_back(normal);
	}

	// Side indices
	for (int i = 0; i < segments; ++i) {
		int top1 = i * 2;
		int bottom1 = top1 + 1;
		int top2 = (i * 2 + 2) % (segments * 2);
		int bottom2 = (i * 2 + 3) % (segments * 2);

		mesh.indices.push_back(top1);
		mesh.indices.push_back(bottom1);
		mesh.indices.push_back(top2);
		mesh.indices.push_back(top2);
		mesh.indices.push_back(bottom1);
		mesh.indices.push_back(bottom2);
	}

	// Add top and bottom caps
	int top_center_index = mesh.vertices.size();
    mesh.vertices.push_back(top_center);
    mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));

    std::vector<int> top_ring_indices;
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * glm::pi<float>() * i / segments;
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        mesh.vertices.push_back(glm::vec3(x, height, z));
        mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
        top_ring_indices.push_back(mesh.vertices.size() - 1);
    }

    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        mesh.indices.push_back(top_center_index);
        mesh.indices.push_back(top_ring_indices[i]);
        mesh.indices.push_back(top_ring_indices[next]);
    }

    int bottom_center_index = mesh.vertices.size();
    mesh.vertices.push_back(bottom_center);
    mesh.normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));

    std::vector<int> bottom_ring_indices;
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * glm::pi<float>() * i / segments;
        float x = radius * cos(angle);
        float z = radius * sin(angle);

        mesh.vertices.push_back(glm::vec3(x, -height, z));
        mesh.normals.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
        bottom_ring_indices.push_back(mesh.vertices.size() - 1);
    }

    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        mesh.indices.push_back(bottom_center_index);
        mesh.indices.push_back(bottom_ring_indices[next]);
        mesh.indices.push_back(bottom_ring_indices[i]);
    }


	return mesh;
}

ES::Engine::Entity CreateCylinder(
	ES::Engine::Core &core,
	const glm::vec3 &position,
	const glm::quat &rotation,
	const glm::vec3 &size)
{
	using namespace JPH;

	glm::vec3 cylinder_scale = glm::vec3(1.0f);

	// Jolt's CylinderShape uses height from center to top, so divide by 2
	float radius = size.x;
	float half_height = size.y;

	auto cylinder_shape_settings = std::make_shared<JPH::CylinderShapeSettings>(half_height, radius);

	ES::Engine::Entity cylinder = core.CreateEntity();
	cylinder.AddComponent<ES::Plugin::Object::Component::Transform>(core, position, cylinder_scale, rotation);
	cylinder.AddComponent<ES::Plugin::Physics::Component::RigidBody3D>(
		core,
		cylinder_shape_settings,
		JPH::EMotionType::Dynamic,
		ES::Plugin::Physics::Utils::Layers::MOVING);

	auto mesh = CreateCylinderMesh(size);
	cylinder.AddComponent<ES::Plugin::Object::Component::Mesh>(core, mesh);

	return cylinder;
}
