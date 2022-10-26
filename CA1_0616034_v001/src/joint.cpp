#include "joint.hpp"

#include "GL/freeglut.h"

#include "rigidbody2D.hpp"
#include "util.hpp"

#include <iostream>

void SpringJoint::ApplyConstriant() const
{
    // TODO
	
	//printf("springjoint:\n");
	//printf("body0's mass: %f, velocity: (%f, %f)\n", m_body0->GetMass(), m_body0->GetVelocity().x, m_body0->GetVelocity().y);
	//printf("body1's mass: %f, velocity: (%f, %f)\n", m_body0->GetMass(), m_body1->GetVelocity().x, m_body1->GetVelocity().y);
	

	float2 StoO = m_body0->GetPosition() - m_body1->GetPosition();
	float2 rv = m_body0->GetVelocity() - m_body1->GetVelocity();
	float length = linalg::length(StoO);
	float2 normalized_StoO=float2(0.0f, 0.0f);
	if (length != 0.0f)
		normalized_StoO = StoO / length;

	float spring = m_stiffness*(length - m_restLength);
	float damper = (m_stiffness / 30)*linalg::dot(rv, normalized_StoO);

	float2 SDForce = (spring + damper)*normalized_StoO;

	//printf("rv: (%f, %f)\n", rv.x, rv.y);
	//printf("normalized_StoO: (%f, %f)\n", normalized_StoO.x, normalized_StoO.y);
	//printf("springForcce: (%f, %f)\n", springForce.x, springForce.y);
	//printf("damperForce: (%f, %f)\n", damperForce.x, damperForce.y);
	//printf("SDForce: (%f, %f)\n", SDForce.x, SDForce.y);
	//system("pause");
	

	//m_body0->AddForce(-SDForce);
	//m_body1->AddForce(SDForce);

}

void SpringJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // red for spring joint
        glColor3f(1, 0, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}

void DistanceJoint::ApplyConstriant() const
{
	// TODO
	float2 StoO = m_body1->GetPosition() - m_body0->GetPosition(); //axis
	float2 rv = m_body1->GetVelocity() - m_body0->GetVelocity(); 
	float length = linalg::length(StoO); //currentDistance
	float2 normalized_StoO = (length == 0.0f) ? StoO : (StoO / length); //unitAxis
	float rv_alongStoO = linalg::dot(rv, normalized_StoO); //relVel 
	float rest_length = m_restLength; //m_distance
	float relDistance = length - rest_length; //relDist
	float dt = m_deltaTime; //dt

	if (relDistance > 0)
	{
		float remove = rv_alongStoO + (relDistance / dt);
		float impulse = 0.0f;
		if (m_body0->GetInvMass() != 0 || m_body1->GetInvMass() != 0)
			impulse = remove / (m_body0->GetInvMass() + m_body1->GetInvMass());

		float2 Impulse = impulse * normalized_StoO;
		m_body0->AddVelocity(Impulse * m_body0->GetInvMass());
		m_body1->AddVelocity(-Impulse * m_body1->GetInvMass());
	}
}

void DistanceJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // green for distance joint
        glColor3f(0, 1, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}