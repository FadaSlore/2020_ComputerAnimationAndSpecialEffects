#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

Manifold AABB::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<const AABB> _shape) const
{
    // TODO

	float2 StoO = _shape->m_body->GetPosition() - m_body->GetPosition(); //Self's position -> Other object's position
	float distance_x, distance_y, overlap = 0.0f; //x_overlap = -distance_x, y_overlap = -distance_y
	bool hit=false;
	float2 normal=float2(0.0f, 0.0f);

	
	if (StoO.x > 0)
		distance_x = (_shape->m_body->GetPosition().x - (_shape->m_extent.x) / 2) - (m_body->GetPosition().x + (m_extent.x) / 2);
	else
		distance_x = (m_body->GetPosition().x - (m_extent.x) / 2) - (_shape->m_body->GetPosition().x + (_shape->m_extent.x) / 2);

	if (StoO.y > 0)
		distance_y = (_shape->m_body->GetPosition().y - (_shape->m_extent.y) / 2) - (m_body->GetPosition().y + (m_extent.y) / 2);
	else
		distance_y = (m_body->GetPosition().y - (m_extent.y) / 2) - (_shape->m_body->GetPosition().y + (_shape->m_extent.y) / 2);

	if (distance_x < 0 && distance_y < 0) // collision happen
	{
		hit = true;
		if ((-distance_x) < (-distance_y)) //x-direction
		{
			overlap = -distance_x;
			normal = (StoO.x > 0) ? float2(1.0f, 0.0f) : float2(-1.0f, 0.0f);
		}
		else //y-direciton
		{
			overlap = -distance_y;
			normal = (StoO.y > 0) ? float2(0.0f, 1.0f) : float2(0.0f, -1.0f);
		}

		/*
		printf("mass of body0: %f, mass of body1: %f\n", m_body->GetMass(), _shape->m_body->GetMass());
		printf("StoO: (%f, %f)\n", StoO.x, StoO.y);
		printf("overlap: %f\n", overlap);
		printf("normal: (%f, %f)\n", normal.x, normal.y);
		system("pause");
		*/
	}
	

	//This is a template return object, you should remove it and return your own Manifold
    return Manifold(
        m_body,
        _shape->m_body,
		normal,
        overlap,
        hit
    );
}

Manifold AABB::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void AABB::Render() const
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}