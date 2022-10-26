#include "integrator.hpp"

#include <algorithm>

#include "scene.hpp"

void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
	// TODO
	const float2 gravity(0.0f, -9.8f);
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		//printf("velocity before: (%f, %f)\n", _bodies[i]->GetVelocity().x, _bodies[i]->GetVelocity().y);
		
		if (_bodies[i]->GetMass()!=0.0f)
		{
			_bodies[i]->AddVelocity(deltaTime*(gravity + (_bodies[i]->GetForce()*_bodies[i]->GetInvMass())));
			_bodies[i]->AddPosition(deltaTime*(_bodies[i]->GetVelocity()));

			//_bodies[i]->AddVelocity(deltaTime*gravity);			
			//_bodies[i]->AddPosition(deltaTime*_bodies[i]->GetVelocity() + ((deltaTime*deltaTime / 2)*(_bodies[i]->GetForce()*_bodies[i]->GetInvMass())));

			//_bodies[i]->AddVelocity(deltaTime*(gravity + (_bodies[i]->GetForce()*_bodies[i]->GetInvMass())));
			//_bodies[i]->AddPosition(deltaTime*_bodies[i]->GetVelocity() + ((deltaTime*deltaTime / 2)*(_bodies[i]->GetForce()*_bodies[i]->GetInvMass())));



			//if (_bodies[i]->GetForce() != float2(0.0f, 0.0f))
			//	printf("number: %d, Force: (%f, %f)\n", ++number, _bodies[i]->GetForce().x, _bodies[i]->GetForce().y);

			//printf("velocity after: (%f, %f)\n", _bodies[i]->GetVelocity().x, _bodies[i]->GetVelocity().y);
			//system("pause");
		}
	}
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    if(scene == nullptr)
    {
        throw std::runtime_error("RungeKuttaFourthIntegrator has no target scene.");
    }

	const float2 gravity(0.0f, -9.8f);
	// TODO

	/*
	// this stores the absolute value of position and velocity
	std::vector<StateStep> currentState(_bodies.size());
	// below four arrays store the delta value of each state
	std::vector<StateStep> deltaK1State(_bodies.size());
	std::vector<StateStep> deltaK2State(_bodies.size());
	std::vector<StateStep> deltaK3State(_bodies.size());
	std::vector<StateStep> deltaK4State(_bodies.size());
	*/

	std::vector<float2> original_position(_bodies.size());
	std::vector<float2> original_velocity(_bodies.size());
	std::vector<float2> original_force(_bodies.size());
	std::vector<float2> k1(_bodies.size());
	std::vector<float2> k2(_bodies.size());
	std::vector<float2> k3(_bodies.size());
	std::vector<float2> k4(_bodies.size());

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		original_position[i] = scene->m_bodies[i]->GetPosition();
		original_velocity[i] = scene->m_bodies[i]->GetVelocity();
		original_force[i] = scene->m_bodies[i]->GetForce();
	}

	//k1
	auto integrator_k1 = std::make_shared<ExplicitEulerIntegrator>();
	auto scene_k1 = std::make_shared<Scene>(
		deltaTime, scene->m_iterations, integrator_k1);
	scene_k1->m_bodies = scene->m_bodies;
	scene_k1->m_joints = scene->m_joints;
	scene_k1->m_manifolds = scene->m_manifolds;
	scene_k1->Step();
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		//printf("original position: (%f, %f)\n", original_position[i].x, original_position[i].y);
		//printf("scene    position: (%f, %f)\n", scene->m_bodies[i]->GetPosition().x, scene->m_bodies[i]->GetPosition().y);
		//printf("scene_k1 position: (%f, %f)\n", scene_k1->m_bodies[i]->GetPosition().x, scene_k1->m_bodies[i]->GetPosition().y);
		//printf("_bodies  position: (%f, %f)\n\n", _bodies[i]->GetPosition().x, _bodies[i]->GetPosition().y);

		k1[i] = scene_k1->m_bodies[i]->GetPosition() - original_position[i];
		scene->m_bodies[i]->SetPosition(original_position[i]);
		scene->m_bodies[i]->SetVelocity(original_velocity[i]);
		scene->m_bodies[i]->SetForce(original_force[i]);
	}
	//system("pause");



	//k2
	auto integrator_k2 = std::make_shared<ExplicitEulerIntegrator>();
	auto scene_k2 = std::make_shared<Scene>(
		(deltaTime) / 2, scene->m_iterations, integrator_k2);
	scene_k2->m_bodies = scene->m_bodies;
	scene_k2->m_joints = scene->m_joints;
	scene_k2->m_manifolds = scene->m_manifolds;
	scene_k2->Step();

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		scene_k2->m_bodies[i]->SetPosition(original_position[i] + (k1[i] / 2));
	}
	scene_k2->m_deltaTime = deltaTime;
	scene_k2->Step();
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		k2[i] = scene_k2->m_bodies[i]->GetPosition() - (original_position[i] + (k1[i] / 2));
		scene->m_bodies[i]->SetPosition(original_position[i]);
		scene->m_bodies[i]->SetVelocity(original_velocity[i]);
		scene->m_bodies[i]->SetForce(original_force[i]);
	}


	//k3
	auto integrator_k3 = std::make_shared<ExplicitEulerIntegrator>();
	auto scene_k3 = std::make_shared<Scene>(
		(deltaTime) / 2, scene->m_iterations, integrator_k3);
	scene_k3->m_bodies = scene->m_bodies;
	scene_k3->m_joints = scene->m_joints;
	scene_k3->m_manifolds = scene->m_manifolds;
	scene_k3->Step();

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		scene_k3->m_bodies[i]->SetPosition(original_position[i] + (k2[i] / 2));
	}
	scene_k3->m_deltaTime = deltaTime;
	scene_k3->Step();
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		k3[i] = scene_k3->m_bodies[i]->GetPosition() - (original_position[i] + (k2[i] / 2));
		scene->m_bodies[i]->SetPosition(original_position[i]);
		scene->m_bodies[i]->SetVelocity(original_velocity[i]);
		scene->m_bodies[i]->SetForce(original_force[i]);
	}


	//k4
	auto integrator_k4 = std::make_shared<ExplicitEulerIntegrator>();
	auto scene_k4 = std::make_shared<Scene>(
		deltaTime, scene->m_iterations, integrator_k4);
	scene_k4->m_bodies = scene->m_bodies;
	scene_k4->m_joints = scene->m_joints;
	scene_k4->m_manifolds = scene->m_manifolds;
	scene_k4->Step();

	for (size_t i = 0; i < _bodies.size(); i++)
	{
		scene_k4->m_bodies[i]->SetPosition(original_position[i] + k3[i]);
	}
	scene_k4->Step();
	for (size_t i = 0; i < _bodies.size(); i++)
	{
		k4[i] = scene_k4->m_bodies[i]->GetPosition() - (original_position[i] + k3[i]);
		scene->m_bodies[i]->SetPosition(original_position[i]);
		scene->m_bodies[i]->SetVelocity(original_velocity[i]);
		scene->m_bodies[i]->SetForce(original_force[i]);
	}


	for (size_t i = 0; i < _bodies.size(); i++)
	{
		if (_bodies[i]->GetInvMass() != 0.0f)
		{
			_bodies[i]->AddPosition((k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6);
			_bodies[i]->AddVelocity(deltaTime*(gravity + (_bodies[i]->GetForce()*_bodies[i]->GetInvMass())));
			//_bodies[i]->AddVelocity(deltaTime*(gravity));
		}
	}
		


}