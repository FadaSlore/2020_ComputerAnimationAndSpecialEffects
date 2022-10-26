#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
	// TODO
	typedef linalg::aliases::float2 float2;
	float2 AABBposition = _a->m_body->GetPosition();
	float2 Circleposition = _b->m_body->GetPosition();
	float2 AABBextent = _a->m_extent;
	AABBextent = AABBextent / 2;
	
	float length = 0.0f;
	float2 closest_point = float2(0.0f, 0.0f);
	bool hit = false, inside = false;
	float2 normal = float2(0.0f, 0.0f);
	float penetration = 0.0f;

	//determine closest point - first step
	if (Circleposition.x > (AABBposition.x + AABBextent.x))
		closest_point.x = AABBposition.x + AABBextent.x;
	else
		closest_point.x = AABBposition.x - AABBextent.x;

	if (Circleposition.y > (AABBposition.y + AABBextent.y))
		closest_point.y = AABBposition.y + AABBextent.y;
	else
		closest_point.y = AABBposition.y - AABBextent.y;

	//determine closest point - after
	if ((Circleposition.x <= (AABBposition.x + AABBextent.x)) && (Circleposition.x >= (AABBposition.x - AABBextent.x))
		&& (Circleposition.y <= (AABBposition.y + AABBextent.y)) && (Circleposition.y >= (AABBposition.y - AABBextent.y)))
	{
		inside = true;
		float2 StoO = Circleposition - AABBposition;
		if (abs(StoO.x) > abs(StoO.y))
			closest_point.y = Circleposition.y;
		else
			closest_point.x = Circleposition.x;
	}
	else if ((Circleposition.x <= (AABBposition.x + AABBextent.x)) && (Circleposition.x >= (AABBposition.x - AABBextent.x)))
		closest_point.x = Circleposition.x;
	else if ((Circleposition.y <= (AABBposition.y + AABBextent.y)) && (Circleposition.y >= (AABBposition.y - AABBextent.y)))
		closest_point.y = Circleposition.y;
	


	length = linalg::length((Circleposition - closest_point));
	if (length < _b->m_radius || inside)
	{
		hit = true;
		if (inside)
		{
			normal = closest_point - Circleposition;
			penetration = _b->m_radius + linalg::length(normal);
		}
		else
		{
			normal = Circleposition - closest_point;
			penetration = _b->m_radius - linalg::length(normal);
		}
		
		if (linalg::length(normal) != 0.0f)
			normal = normal / linalg::length(normal);
	}
	

	//This is a template return object, you should remove it and return your own Manifold
	return Manifold(
		_a->m_body,
		_b->m_body,
		normal,
        penetration,
        hit
    );
}