//
//	CollisionUtil.cpp
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

#include "CollisionUtil.h"


class WorldAABBQuery : public b2QueryCallback
{
public:	
	WorldAABBQuery(b2Fixture** results, int maxResults, const QueryFilter& filter)
	: mFilter(filter)
	, mResults(results)
	, mResultCount(0)
	, mMaxResults(maxResults)
	{
	}
	
	// Called for each fixture found in the query AABB.
	// return false to terminate the query.
	bool ReportFixture(b2Fixture* fixture)
	{
		if( mFilter.test(fixture->GetFilterData()) )
		{
			mResults[mResultCount] = fixture;
			mResultCount++;
		}
		
		return mResultCount < mMaxResults;
	}
	
	QueryFilter mFilter;		
	b2Fixture** mResults;
	int mResultCount;	
	int mMaxResults;	
};

int QueryAABB(b2World* world, const b2AABB& aabb, const QueryFilter& filter, b2Fixture** results, int maxResults )
{	
	WorldAABBQuery cb(results, maxResults, filter);
	world->QueryAABB(&cb, aabb);
	return cb.mResultCount;
}


////////////////////////////////////////////////////////////////////////////
// raycasting

class RayCastCB : public b2RayCastCallback
{
public:
	RayCastCB( RayCastResult* results, int maxResults, const QueryFilter& filter )
	: mResults(results)
	, mResultCount(0)
	, mMaxResults(maxResults)
	, mFilter(filter)
	{
		assert(results != NULL);
	}
	
	RayCastResult* mResults;
	int mResultCount;	
	int mMaxResults;
	QueryFilter mFilter;
	
	
	/// Called for each fixture found in the query. You control how the ray cast
	/// proceeds by returning a float:
	/// return -1: ignore this fixture and continue
	/// return 0: terminate the ray cast
	/// return fraction: clip the ray to this point
	/// return 1: don't clip the ray and continue
	/// @param fixture the fixture hit by the ray
	/// @param point the point of initial intersection
	/// @param normal the normal vector at the point of intersection
	/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
	/// closest hit, 1 to continue	
	virtual float32 ReportFixture( b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		bool ignore = mFilter.ignored != NULL && mFilter.ignored == fixture->GetBody()->GetUserData();
		
		if( !ignore && mFilter.test(fixture->GetFilterData()) )
		{
			RayCastResult& result = mResults[mResultCount];
			
			result.fixture = fixture;
			result.point = point;
			result.normal = normal;
			result.fraction = fraction;
			
			mResultCount++;
		}
		
		return mResultCount < mMaxResults ? 1 : 0;
	}
};

int CollideRay( b2World* world, const b2Vec2& from, const b2Vec2& to, const QueryFilter& filter, RayCastResult* results, int maxResults )
{
	RayCastCB cb(results, maxResults, filter);
	world->RayCast(&cb, from, to);
	return cb.mResultCount;
}





class RayCastClosestCB : public b2RayCastCallback
{
public:
	RayCastClosestCB( RayCastResult* result, const QueryFilter& filter )
	: mResult(result)
	, mFilter(filter)
	, mFraction(1.f)
	, mHit(false)
	{
	}
	
	RayCastResult* mResult;
	QueryFilter mFilter;
	int mFraction;
	bool mHit;
	
	virtual float32 ReportFixture( b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		bool ignore = mFilter.ignored != NULL && mFilter.ignored == fixture->GetBody()->GetUserData();		
		
		// apendley - don't actually know if this is necessary.  we may be alright just returning the first hit
		if( !ignore && mFilter.test(fixture->GetFilterData()) && fraction <= mFraction )
		{
			mHit = true;
			mFraction = fraction;
			
			if( mResult )
			{
				mResult->fixture = fixture;
				mResult->point = point;
				mResult->normal = normal;
				mResult->fraction = fraction;
			}
			
			return 1;
		}
		
		return -1;
	}
};


bool CollideRayClosest( b2World* world, const b2Vec2& from, const b2Vec2& to, const QueryFilter& filter, RayCastResult* result)
{
	RayCastClosestCB cb(result, filter);
	world->RayCast(&cb, from, to);
	return cb.mHit;	
}





////////////////////////////////////////////////////////////////////////////
// shape casting (swept collision test)

// sweep a shape against another known shape in the world
bool CollideSwept( b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, b2Shape* shapeOther, const b2Transform& xformOther, const b2Vec2& localCenterOther, const b2Vec2& motion, ShapeCastResult* result )
{
	b2Sweep sweep;
	
	// build sweep structures
	sweep.a0 = sweep.a = xform.GetAngle();
	sweep.localCenter = localCenter;
	sweep.c0 = xform.position;
	sweep.c = xform.position + motion;
	
	b2Sweep otherSweep;
	b2Transform otherTransform = xformOther;
	otherSweep.localCenter = localCenterOther;
	otherSweep.a0 = otherSweep.a = xformOther.GetAngle();
	otherSweep.c0 = otherSweep.c = b2Mul(otherTransform, otherSweep.localCenter);
	
	// now let's get a TOI on the collision
	b2TOIInput toiInput;
	toiInput.proxyA.Set(shape, 0);
	toiInput.proxyB.Set(shapeOther, 0);
	toiInput.sweepA = sweep;
	toiInput.sweepB = otherSweep;
	toiInput.tMax = 1.0f;
	
	b2TOIOutput toiOutput;
	b2TimeOfImpact(&toiOutput, &toiInput);		
	
	// i want to be aware of anything unexpected...
	assert(toiOutput.state == b2TOIOutput::e_touching || toiOutput.state == b2TOIOutput::e_separated || toiOutput.state == b2TOIOutput::e_overlapped);
	
	if( toiOutput.state == b2TOIOutput::e_touching )
	{
		// get the collision info to hand back to the caller
		b2DistanceInput distInput;
		distInput.proxyA.Set(shape, 0);
		distInput.proxyB.Set(shapeOther, 0);
		sweep.GetTransform(&distInput.transformA, toiOutput.t);				
		distInput.transformB = xformOther;
		distInput.useRadii = false;
		
		b2SimplexCache cache;
		cache.count = 0;
		
		b2DistanceOutput distOutput;
		b2Distance(&distOutput, &cache, &distInput);
		
		if( result != NULL )
		{
			// calculate collision normal
			b2Vec2 normal = distOutput.pointA - distOutput.pointB;
			normal *= (1.f / distOutput.distance);
			result->normal = normal;
			
			// hmm..guess i'll return the collision point on the other fixture.  we could average the points also.
			// or some other third way that is the actual correct way to do it
			result->contactPoint = distOutput.pointB;
			//b2Vec2 intersection = distanceOutput.pointA + shape->m_radius * normal;			
			
			// reuse result from calculated b2DistanceInput
			result->toi = distInput.transformA.position;
			
			// no fixture
			result->fixture = NULL;
		}
		
		return true;		
	}		
	
	return false;	
	
}

bool CollideSwept( b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, b2Fixture* otherFixture, const b2Vec2& motion, ShapeCastResult* result )
{
	return CollideSwept(shape, xform, localCenter, 
						otherFixture->GetShape(), 
						otherFixture->GetBody()->GetTransform(), 
						otherFixture->GetBody()->GetLocalCenter(),
						motion, result);	
}

int CollideSwept( b2World* world, b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, const b2Vec2& motion, const QueryFilter& filter, ShapeCastResult* results, int maxResults)
{
	int resultCount = 0;
	
	b2AABB sweptAABB;
	shape->ComputeAABB(&sweptAABB, xform, 0);
	
	b2AABB toAABB;
	toAABB.lowerBound = sweptAABB.lowerBound + motion; 
	toAABB.upperBound = sweptAABB.upperBound + motion;
	sweptAABB.Combine(sweptAABB, toAABB);
	
	// first do the AABB query to collect any fixtures along our swept AABB
	static int const kMaxAABBResults = 16;
	b2Fixture* queryResults[kMaxAABBResults];
	int aabbCount = QueryAABB(world, sweptAABB, filter, queryResults, kMaxAABBResults);
	
	for( int fixtureIdx = 0; fixtureIdx < aabbCount; ++fixtureIdx )
	{
		b2Fixture* otherFixture = queryResults[fixtureIdx];
		b2Body* otherBody = otherFixture->GetBody();
		
		if( otherBody->GetUserData() != NULL && otherBody->GetUserData() == filter.ignored )
			continue;
		
		// build sweep structures
		b2Sweep sweep;
		sweep.a0 = sweep.a = xform.GetAngle();
		sweep.localCenter = localCenter;
		sweep.c0 = xform.position;
		sweep.c = xform.position + motion;
		
		b2Sweep otherSweep;
		b2Transform otherTransform = otherBody->GetTransform();
		otherSweep.localCenter = otherBody->GetLocalCenter();
		otherSweep.a0 = otherSweep.a = otherBody->GetAngle();
		otherSweep.c0 = otherSweep.c = b2Mul(otherTransform, otherSweep.localCenter);
		
		// now let's get a TOI on the collision
		b2TOIInput toiInput;
		toiInput.proxyA.Set(shape, 0);
		toiInput.proxyB.Set(otherFixture->GetShape(), 0);
		toiInput.sweepA = sweep;
		toiInput.sweepB = otherSweep;
		toiInput.tMax = 1.0f;
		
		b2TOIOutput toiOutput;
		b2TimeOfImpact(&toiOutput, &toiInput);		
		
		// i want to be aware of anything unexpected...
		assert(toiOutput.state == b2TOIOutput::e_touching || toiOutput.state == b2TOIOutput::e_separated);
		
		if( toiOutput.state == b2TOIOutput::e_touching )
		{
			// get the collision info to hand back to the caller
			b2DistanceInput distInput;
			distInput.proxyA.Set(shape, 0);
			distInput.proxyB.Set(otherFixture->GetShape(), 0);
			sweep.GetTransform(&distInput.transformA, toiOutput.t);				
			distInput.transformB = otherFixture->GetBody()->GetTransform();
			distInput.useRadii = false;
			
			b2SimplexCache cache;
			cache.count = 0;
			
			b2DistanceOutput distOutput;
			b2Distance(&distOutput, &cache, &distInput);
			
			ShapeCastResult& result = results[resultCount];

			// calculate collision normal
			b2Vec2 normal = distOutput.pointA - distOutput.pointB;
			normal *= (1.f / distOutput.distance);
			result.normal = normal;
			
			// hmm..guess i'll return the collision point on the other fixture.  we could average the points also.
			// or some other third way that is the actual correct way to do it
			result.contactPoint = distOutput.pointB;
			//b2Vec2 intersection = distanceOutput.pointA + shape->m_radius * normal;			
			
			// reuse result from calculated b2DistanceInput
			result.toi = distInput.transformA.position;
			
			result.fixture = otherFixture;
			
			resultCount++;
		}
	}
	
	return resultCount;
}

bool CollideSweptClosest( b2World* world, b2Shape* shape, const b2Transform& xform, const b2Vec2& localCenter, const b2Vec2& motion, const QueryFilter& filter, ShapeCastResult* result )
{
	b2AABB sweptAABB;
	shape->ComputeAABB(&sweptAABB, xform, 0);
	
	b2AABB toAABB;
	toAABB.lowerBound = sweptAABB.lowerBound + motion; 
	toAABB.upperBound = sweptAABB.upperBound + motion;
	sweptAABB.Combine(sweptAABB, toAABB);
	
	// first do the AABB query to collect any fixtures along our swept AABB
	static int const kMaxAABBResults = 16;
	b2Fixture* queryResults[kMaxAABBResults];
	int aabbCount = QueryAABB(world, sweptAABB, filter, queryResults, kMaxAABBResults);
	
	b2Fixture* closest = NULL;
	float smallestTOI = 1.0f;
	b2Sweep sweep;
	
	for( int fixtureIdx = 0; fixtureIdx < aabbCount; ++fixtureIdx )
	{
		b2Fixture* otherFixture = queryResults[fixtureIdx];
		b2Body* otherBody = otherFixture->GetBody();
		
		if( otherBody->GetUserData() != NULL && otherBody->GetUserData() == filter.ignored )
			continue;		
		
		// build sweep structures
		sweep.a0 = sweep.a = xform.GetAngle();
		sweep.localCenter = localCenter;
		sweep.c0 = xform.position;
		sweep.c = xform.position + motion;
		
		b2Sweep otherSweep;
		b2Transform otherTransform = otherBody->GetTransform();
		otherSweep.localCenter = otherBody->GetLocalCenter();
		otherSweep.a0 = otherSweep.a = otherBody->GetAngle();
		otherSweep.c0 = otherSweep.c = b2Mul(otherTransform, otherSweep.localCenter);
		
		// now let's get a TOI on the collision
		b2TOIInput toiInput;
		toiInput.proxyA.Set(shape, 0);
		toiInput.proxyB.Set(otherFixture->GetShape(), 0);
		toiInput.sweepA = sweep;
		toiInput.sweepB = otherSweep;
		toiInput.tMax = 1.0f;
		
		b2TOIOutput toiOutput;
		b2TimeOfImpact(&toiOutput, &toiInput);		
		
		// i want to be aware of anything unexpected...
		assert(toiOutput.state == b2TOIOutput::e_touching || toiOutput.state == b2TOIOutput::e_separated || toiOutput.state == b2TOIOutput::e_overlapped);
		
		if( toiOutput.state == b2TOIOutput::e_touching && toiOutput.t < smallestTOI )
		{
			closest = otherFixture;
			smallestTOI = toiOutput.t;
		}		
	}
	
	if( closest != NULL )
	{
		// get the collision info to hand back to the caller
		b2DistanceInput distInput;
		distInput.proxyA.Set(shape, 0);
		distInput.proxyB.Set(closest->GetShape(), 0);
		sweep.GetTransform(&distInput.transformA, smallestTOI);				
		distInput.transformB = closest->GetBody()->GetTransform();
		distInput.useRadii = false;
		
		b2SimplexCache cache;
		cache.count = 0;
		
		b2DistanceOutput distOutput;
		b2Distance(&distOutput, &cache, &distInput);
		
		if( result != NULL )
		{
			// calculate collision normal
			b2Vec2 normal = distOutput.pointA - distOutput.pointB;
			normal *= (1.f / distOutput.distance);
			result->normal = normal;
			
			// hmm..guess i'll return the collision point on the other fixture.  we could average the points also.
			// or some other third way that is the actual correct way to do it
			result->contactPoint = distOutput.pointB;
			//b2Vec2 intersection = distanceOutput.pointA + shape->m_radius * normal;			
			
			// reuse result from calculated b2DistanceInput
			result->toi = distInput.transformA.position;
			
			result->fixture = closest;
		}
		
		return true;
	}	
	
	return false;
}