#ifndef DOUBLE_ARC_H
#define DOUBLE_ARC_H
/******************************************************************************
  Copyright (c) 2014 Ryan Juckett
  http://www.ryanjuckett.com/
  
  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.
  
  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:
  
  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  
  3. This notice may not be removed or altered from any source
     distribution.
==============================================================================
  Modificated by Rafael Jin.
******************************************************************************/


//******************************************************************************
//******************************************************************************

#include <stdio.h>
#include <cmath>
#include <assert.h>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

static float const pi = 3.1415926;

struct tVec3
{
    float m_x;
    float m_y;
    float m_z;
};

//******************************************************************************
// Compute the dot product of two vectors.
//******************************************************************************
float Vec_DotProduct(const tVec3 & lhs, const tVec3 & rhs);
 
//******************************************************************************
// Compute the cross product of two vectors.
//******************************************************************************
void Vec_CrossProduct(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs);
 
//******************************************************************************
// Compute the sum of two vectors.
//******************************************************************************
void Vec_Add(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs);
 
//******************************************************************************
// Compute the difference of two vectors.
//******************************************************************************
void Vec_Subtract(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs);
 
//******************************************************************************
// Compute a scaled vector.
//******************************************************************************
void Vec_Scale(tVec3 * pResult, const tVec3 & lhs, float rhs);
 
//******************************************************************************
// Add a vector to a scaled vector.
//******************************************************************************
void Vec_AddScaled(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs, float rhsScale);
 
//******************************************************************************
// Compute the magnitude of a vector.
//******************************************************************************
float Vec_Magnitude(const tVec3 & lhs);
 
//******************************************************************************
// Check if the vector length is within epsilon of 1
//******************************************************************************
bool Vec_IsNormalized_Eps(const tVec3 & value, float epsilon);
 
//******************************************************************************
// Return 1 or -1 based on the sign of a real number.
//******************************************************************************
inline float Sign(float val);

//******************************************************************************
// Information about an arc used in biarc interpolation. Use 
// Vec_BiarcInterp_ComputeArcs to compute the values and use Vec_BiarcInterp
// to interpolate along the arc pair.
//******************************************************************************
struct tBiarcInterp_Arc
{
    tVec3   m_center;   // center of the circle (or line)
    tVec3   m_axis1;    // vector from center to the end point
    tVec3   m_axis2;    // vector from center edge perpendicular to axis1
    float   m_radius;   // radius of the circle (zero for lines)
    float   m_angle;    // angle to rotate from axis1 towards axis2
    float   m_arcLen;   // distance along the arc
};

//******************************************************************************
// Compute a single arc based on an end point and the vector from the endpoint
// to connection point. 
//******************************************************************************
void BiarcInterp_ComputeArc
(
    tVec3 *         pCenter,    // Out: Center of the circle or straight line.
    float *         pRadius,    // Out: Zero for straight lines
    float *         pAngle,     // Out: Angle of the arc
    const tVec3 &   point,
    const tVec3 &   tangent,
    const tVec3 &   pointToMid
);


//******************************************************************************
// Compute a pair of arcs to pass into Vec_BiarcInterp
// http://www.ryanjuckett.com/programming/biarc-interpolation/
//******************************************************************************
void BiarcInterp_ComputeArcs
(
    tBiarcInterp_Arc *  pArc1,
    tBiarcInterp_Arc *  pArc2,
    const tVec3 &   p1,     // start position
    const tVec3 &   t1,     // start tangent
    const tVec3 &   p2,     // end position
    const tVec3 &   t2      // end tangent
);

//******************************************************************************
// Use a biarc to interpolate between two points such that the interpolation
// direction aligns with associated tangents.
// http://www.ryanjuckett.com/programming/biarc-interpolation/
//******************************************************************************
void BiarcInterp
(
    tVec3 *                     pResult,    // interpolated point
    const tBiarcInterp_Arc &    arc1,
    const tBiarcInterp_Arc &    arc2,
    float                       frac        // [0,1] fraction along the biarc
);

void generateArcPath(Motor *motor,Location &location,cv::Point2d dest_loc,cv::Point2d dest_dir,float speed);

#endif