#include "manifold.hpp"

#include <iostream>

Manifold::Manifold(
    std::shared_ptr<RigidBody2D> _body0, 
    std::shared_ptr<RigidBody2D> _body1,
    float2 _normal,
    float _penetration,
    bool _isHit)
    : m_body0(_body0), m_body1(_body1), m_normal(_normal),
      m_penetration(_penetration), m_isHit(_isHit)
    {}

void Manifold::Resolve() const
{
    float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
	// TODO
	if (m_isHit)
	{
		float2 rv = m_body1->GetVelocity()-m_body0->GetVelocity(); //relative velocity
		float rv_alongN = linalg::dot(rv, m_normal); //relative velocity along normal
		if (rv_alongN > 0) //far from each other
			return;

		float impulse = 0.0f;
		if (m_body0->GetInvMass() != 0.0f || m_body1->GetInvMass() != 0.0f)
			impulse = (-(1 + e)*rv_alongN) / (m_body0->GetInvMass() + m_body1->GetInvMass()); //normal impulse scalar
		float2 i_alongN = impulse * m_normal; //impulse

		m_body0->AddVelocity(-(i_alongN*m_body0->GetInvMass()));
		m_body1->AddVelocity((i_alongN*m_body1->GetInvMass()));

		/*
		printf("After resolution:\n");
		printf("body0's mass: %f, velocity: (%f, %f)\n", m_body0->GetMass(), m_body0->GetVelocity().x, m_body0->GetVelocity().y);
		printf("body1's mass: %f, velocity: (%f, %f)\n", m_body0->GetMass(), m_body1->GetVelocity().x, m_body1->GetVelocity().y);
		*/

		//fiction
		rv = m_body1->GetVelocity() - m_body0->GetVelocity(); //recaculate relative velocity
		float2 tangent = rv - linalg::dot(rv, m_normal)*m_normal;
		if (linalg::length(tangent) != 0.0f)
			tangent = (tangent / linalg::length(tangent));
		float rv_alongT = linalg::dot(rv, tangent); //relative velocity along tangent direction

		float staticFriction = sqrt(m_body0->m_staticFriction*m_body1->m_staticFriction);
		float dynamicFriction = sqrt(m_body0->m_dynamicFriction*m_body1->m_dynamicFriction);
		
		float frictionImpulse = 0.0f;
		if (m_body0->GetInvMass() != 0.0f || m_body1->GetInvMass() != 0.0f)
			frictionImpulse = rv_alongT / (m_body0->GetInvMass() + m_body1->GetInvMass()); // friction impulse scalar
		float2 frictionI_alongT; //friction impluse

		if (abs(frictionImpulse) < (abs(impulse)*staticFriction))
			frictionI_alongT = frictionImpulse * tangent; //static friction
		else
			frictionI_alongT = impulse * dynamicFriction*tangent; //dynamic friction


		/*
		printf("rv: (%f, %f)\n", rv.x, rv.y);
		printf("dot of rv and normal: %f\n", linalg::dot(rv, m_normal));
		printf("normal: (%f, %f)\n", m_normal.x, m_normal.y);
		printf("fiction_alongT: (%f, %f)\n", frictionI_alongT.x, frictionI_alongT.y);
		printf("frictionImpulse: %f\n", frictionImpulse);
		printf("impluse: %f\n", impulse);
		printf("tangent: (%f, %f)\n", tangent.x, tangent.y);
		system("pause");
		*/

		m_body0->AddVelocity((frictionI_alongT*m_body0->GetInvMass()));
		m_body1->AddVelocity(-(frictionI_alongT*m_body1->GetInvMass()));
	}
}

void Manifold::PositionalCorrection() const
{
    const float percent = 0.4f; // usually 20% to 80%, when fps is 1/60
    const float slop = 0.01f;

	const float inv_mass_a = m_body0->GetInvMass();
	const float inv_mass_b = m_body1->GetInvMass();

    if(inv_mass_a == 0.0f && inv_mass_b == 0.0f)
        return;

    float2 correction = 
        (std::max( m_penetration - slop, 0.0f ) / (inv_mass_a + inv_mass_b))
        * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;

	if (0) //(correction != float2(0.0f, 0.0f))
	{
		printf("correction: (%f, %f)\n", correction.x, correction.y);
		system("pause");
	}
		
}