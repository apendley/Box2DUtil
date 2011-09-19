//
//	Box2dUtil.h
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



#ifndef _BOX2DUTIL_H_INCLUDED
#define _BOX2DUTIL_H_INCLUDED

#include "Box2D.h"
#include "ccMacros.h"

#include <CoreGraphics/CGGeometry.h>

// Pixel to meters ratio. Box2D uses metres as the unit for measurement.
// This ratio defines how many pixels correspond to one Box2D meter
// Box2D is optimized for objects of 1x1 meter so it makes sense
// to define the ratio so that your most common object type is 1x1 meter.
#define kPhysicsPTMRatio			(32.0f) 
#define kPhysicsPTMRatioInv			(1.0f/kPhysicsPTMRatio)


//
// b2Vec2 does not overload these operators, so I am overloading them here.
// I will try to add these to the Box2D distribution, as I think they are very helpful.
// in the meantime, I'll just define them here for my own use
//
inline b2Vec2 operator * (const b2Vec2& v, const float32 a )
{
	return b2Vec2(v.x * a, v.y * a);
}

inline b2Vec2 operator / (const b2Vec2& v, const float32 a )
{
	float32 inv( 1.0f / a );
	return b2Vec2(v.x * inv, v.y * inv);
}


//
// conversion functions;
// Box2D deals in meters, while Cocos2D deals in points.  this will convert between the two
//

inline b2Vec2 tob2(const b2Vec2& v)
{
	return v * kPhysicsPTMRatioInv;
}

inline b2Vec2 fromb2(const b2Vec2& v)
{
	return v * kPhysicsPTMRatio;
}

inline b2Vec2 tob2(float32 x, float32 y)
{
	return tob2(b2Vec2(x, y));
}

inline b2Vec2 fromb2(float32 x, float32 y)
{
	return fromb2(b2Vec2(x, y));
}

inline float32 tob2(float32 v)
{
	return v * kPhysicsPTMRatioInv;
}

inline float32 fromb2(float32 v)
{
	return v * kPhysicsPTMRatio;
}

inline b2Vec2 ccptob2(const CGPoint& point, bool scale = true )
{
	if( scale )
	{
		return tob2(point.x, point.y);
	}
	
	return b2Vec2(point.x, point.y);	
}

inline b2Vec2 ccstob2(const CGSize& size, bool scale = true )
{
	if( scale )
	{
		return tob2(size.width, size.height);
	}
	
	return b2Vec2(size.width, size.height);
}

inline CGPoint b2toccp(const b2Vec2& point, bool scale = true)
{
	if( scale )
	{
		return CGPointMake(fromb2(point.x), fromb2(point.y));
	}
	
	return CGPointMake(point.x, point.y);	
}

inline float32 getAngle(const b2Vec2& v)
{
	return b2Atan2(v.y, v.x);
}

// returns an AABB that encapsulates all of the fixtures in specified body
inline b2AABB bodyAABB(b2Body* body)
{
	b2AABB aabb;
	
	bool first = true;
	
	for( b2Fixture* fixture = body->GetFixtureList(); fixture; fixture = fixture->GetNext() )
	{
		b2AABB shapeAABB;
		fixture->GetShape()->ComputeAABB(&shapeAABB, body->GetTransform(), 0);
		
		
		if( first )
		{
			first = false;
			aabb = shapeAABB;
		}
		else
		{
			aabb.Combine(aabb, shapeAABB);
		}
	}
	
	return aabb;
}

#endif
