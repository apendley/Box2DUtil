//
//	CollisionUtil.h
//
//	Created by Aaron Pendley on 4/9/10.
//	Copyright 2010 Aaron Pendley.
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
// 
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
// 
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
//


#ifndef _COLLISIONUTIL_H_INCLUDED_
#define _COLLISIONUTIL_H_INCLUDED_

#include "Box2D.h"


// filter data for collision util queries.
struct QueryFilter
{
	static const unsigned int kDefaultMask = 0xFFFF;
	
	QueryFilter(unsigned short maskBits = kDefaultMask, unsigned short categoryBits = 0, void* ignore = NULL)
	: maskFilter(maskBits)
	, categoryFilter(categoryBits)
	, ignored(ignore)
	{
	}
	
	inline bool test(unsigned short maskBits, unsigned short categoryBits = 0)
	{
		return (maskBits & maskFilter) != 0 || (categoryBits & categoryFilter) != 0 ;
	}
	
	inline bool test(const b2Filter& filter)
	{
		return test( filter.maskBits, filter.categoryBits );
	}
	
	unsigned short maskFilter;
	unsigned short categoryFilter;
	void* ignored;
};


////////////////////////////////////////////////////////////////////////////
// AABB query

// collects all fixtures that match the filter category intersecting the specified AABB
int QueryAABB(b2World* world, const b2AABB& aabb, const QueryFilter& filter, b2Fixture** results, int maxResults );




////////////////////////////////////////////////////////////////////////////
// ray casting

struct RayCastResult
{
	RayCastResult():fixture(NULL), fraction(0){}
	b2Fixture* fixture;
	b2Vec2 point;
	b2Vec2 normal;
	float fraction;
};

// collects all collisions along a ray that match the specified filter category
int CollideRay( b2World* world, const b2Vec2& from, const b2Vec2& to, const QueryFilter& filter, RayCastResult* results, int maxResults );

// returns the closest collision along specified ray
bool CollideRayClosest( b2World* world, const b2Vec2& from, const b2Vec2& to, const QueryFilter& filter, RayCastResult* result);



////////////////////////////////////////////////////////////////////////////
// shape casting (swept collision test)

struct ShapeCastResult
{
	b2Vec2	normal;
	b2Vec2	contactPoint;
	b2Vec2	toi;
	b2Fixture* fixture;
};


// sweep a shape against another known shape
bool CollideSwept( b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, b2Shape* shapeOther, const b2Transform& xformOther, const b2Vec2& localCenterOther, const b2Vec2& motion, ShapeCastResult* result );

// sweep a shape against another known shape in the world
bool CollideSwept( b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, b2Fixture* otherFixture, const b2Vec2& motion, ShapeCastResult* result );

// return all collisions along the path of a swept shape
int CollideSwept( b2World* world, b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, const b2Vec2& motion, const QueryFilter& filter, ShapeCastResult* results, int maxResults );

// returns the closest collision along the path of a swept shape
bool CollideSweptClosest( b2World* world, b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, const b2Vec2& motion, const QueryFilter& filter, ShapeCastResult* result );

#endif