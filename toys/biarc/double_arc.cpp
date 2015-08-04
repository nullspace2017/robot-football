 #include "double_arc.h"

float Vec_DotProduct(const tVec3 & lhs, const tVec3 & rhs)
{
    return lhs.m_x*rhs.m_x + lhs.m_y*rhs.m_y + lhs.m_z*rhs.m_z;
}
 
void Vec_CrossProduct(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs)
{
    float x = lhs.m_y*rhs.m_z - lhs.m_z*rhs.m_y;
    float y = lhs.m_z*rhs.m_x - lhs.m_x*rhs.m_z;
    float z = lhs.m_x*rhs.m_y - lhs.m_y*rhs.m_x;
 
    pResult->m_x = x;
    pResult->m_y = y;
    pResult->m_z = z;
}
 
void Vec_Add(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs)
{
    pResult->m_x = lhs.m_x + rhs.m_x;
    pResult->m_y = lhs.m_y + rhs.m_y;
    pResult->m_z = lhs.m_z + rhs.m_z;
}
 
void Vec_Subtract(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs)
{
    pResult->m_x = lhs.m_x - rhs.m_x;
    pResult->m_y = lhs.m_y - rhs.m_y;
    pResult->m_z = lhs.m_z - rhs.m_z;
}
 
void Vec_Scale(tVec3 * pResult, const tVec3 & lhs, float rhs)
{
    pResult->m_x = lhs.m_x*rhs;
    pResult->m_y = lhs.m_y*rhs;
    pResult->m_z = lhs.m_z*rhs;
}
 
void Vec_AddScaled(tVec3 * pResult, const tVec3 & lhs, const tVec3 & rhs, float rhsScale)
{
    pResult->m_x = lhs.m_x + rhs.m_x*rhsScale;
    pResult->m_y = lhs.m_y + rhs.m_y*rhsScale;
    pResult->m_z = lhs.m_z + rhs.m_z*rhsScale;
}
 
float Vec_Magnitude(const tVec3 & lhs)
{
    return sqrt(Vec_DotProduct(lhs,lhs));
}
 
bool Vec_IsNormalized_Eps(const tVec3 & value, float epsilon)
{
    const float sqrMag = Vec_DotProduct(value,value);
    return      sqrMag >= (1.0f - epsilon)*(1.0f - epsilon)
            &&  sqrMag <= (1.0f + epsilon)*(1.0f + epsilon);
}
 
inline float Sign(float val)
{
    return (val < 0.0f) ? -1.0f : 1.0f;
}

void BiarcInterp_ComputeArc
(
    tVec3 *         pCenter,    // Out: Center of the circle or straight line.
    float *         pRadius,    // Out: Zero for straight lines //!infinity check
    float *         pAngle,     // Out: Angle of the arc
    const tVec3 &   point,
    const tVec3 &   tangent,
    const tVec3 &   pointToMid
)
{
    // assume that the tangent is normalized
    assert( Vec_IsNormalized_Eps(tangent,0.01f) );
 
    const float c_Epsilon = 0.0001f;
 
    // compute the normal to the arc plane
    tVec3 normal;
    Vec_CrossProduct(&normal, pointToMid, tangent);
 
    // Compute an axis within the arc plane that is perpendicular to the tangent.
    // This will be coliniear with the vector from the center to the end point.
    tVec3 perpAxis;
    Vec_CrossProduct(&perpAxis, tangent, normal);
 
    const float denominator = 2.0f * Vec_DotProduct(perpAxis, pointToMid);
             
    if (fabs(denominator) < c_Epsilon)
    {
        // The radius is infinite, so use a straight line. Place the center point in the
        // middle of the line.
        Vec_AddScaled(pCenter, point, pointToMid, 0.5f);
        *pRadius = 0.0f;
        *pAngle  = 0.0f;
    }
    else
    {
        // Compute the distance to the center along perpAxis
        const float centerDist = Vec_DotProduct(pointToMid,pointToMid) / denominator;
        Vec_AddScaled(pCenter, point, perpAxis, centerDist);
 
        // Compute the radius in absolute units
        const float perpAxisMag = Vec_Magnitude(perpAxis);
        const float radius = fabs(centerDist*perpAxisMag);
 
        // Compute the arc angle
        float angle;
        if (radius < c_Epsilon)
        {
            angle = 0.0f;
        }
        else
        {
            const float invRadius = 1.0f / radius;
                     
            // Compute normalized directions from the center to the connection point
            // and from the center to the end point.
            tVec3 centerToMidDir;
            tVec3 centerToEndDir;
                     
            Vec_Subtract(&centerToMidDir, point, *pCenter);
            Vec_Scale(&centerToEndDir, centerToMidDir, invRadius);
 
            Vec_Add(&centerToMidDir, centerToMidDir, pointToMid);
            Vec_Scale(&centerToMidDir, centerToMidDir, invRadius);
 
            // Compute the rotation direction
            const float twist = Vec_DotProduct(perpAxis, pointToMid);
 
            // Compute angle.
            angle = acosf( Vec_DotProduct(centerToEndDir,centerToMidDir) ) * Sign(twist);
        }
 
        // output the radius and angle
        *pRadius = radius;
        *pAngle  = angle;
    }
}


void BiarcInterp_ComputeArcs
(
    tBiarcInterp_Arc *  pArc1,
    tBiarcInterp_Arc *  pArc2,
    const tVec3 &   p1,     // start position
    const tVec3 &   t1,     // start tangent
    const tVec3 &   p2,     // end position
    const tVec3 &   t2      // end tangent
)
{
    assert( Vec_IsNormalized_Eps(t1,0.01f) );
    assert( Vec_IsNormalized_Eps(t2,0.01f) );
 
    const float c_Pi        = 3.1415926535897932384626433832795f;
    const float c_2Pi       = 6.2831853071795864769252867665590f;
    const float c_Epsilon   = 0.0001f;
 
    tVec3 v;
    Vec_Subtract(&v, p2, p1);
 
    const float vDotV = Vec_DotProduct(v,v);
 
    // if the control points are equal, we don't need to interpolate
    if (vDotV < c_Epsilon)
    {
        pArc1->m_center = p1;
        pArc2->m_radius = 0.0f;
        pArc1->m_axis1 = v;
        pArc1->m_axis2 = v;
        pArc1->m_angle = 0.0f;
        pArc1->m_arcLen = 0.0f;
 
        pArc2->m_center = p1;
        pArc2->m_radius = 0.0f;
        pArc2->m_axis1 = v;
        pArc2->m_axis2 = v;
        pArc2->m_angle = 0.0f;
        pArc2->m_arcLen = 0.0f;
        return;
    }
 
    // computw the denominator for the quadratic formula
    tVec3 t;
    Vec_Add(&t, t1, t2);
 
    const float vDotT       = Vec_DotProduct(v,t);
    const float t1DotT2     = Vec_DotProduct(t1,t2);
    const float denominator = 2.0f*(1.0f - t1DotT2);
 
    // if the quadratic formula denominator is zero, the tangents are equal and we need a special case
    float d;
    if (denominator < c_Epsilon)
    {
        const float vDotT2 = Vec_DotProduct(v,t2);
                 
        // if the special case d is infinity, the only solution is to interpolate across two semicircles
        if ( fabs(vDotT2) < c_Epsilon )
        {
            const float vMag = sqrt(vDotV);
            const float invVMagSqr = 1.0f / vDotV;
 
            // compute the normal to the plane containing the arcs
            // (this has length vMag)
            tVec3 planeNormal;
            Vec_CrossProduct(&planeNormal, v, t2);
 
            // compute the axis perpendicular to the tangent direction and aligned with the circles
            // (this has length vMag*vMag)
            tVec3 perpAxis;
            Vec_CrossProduct(&perpAxis, planeNormal, v);
 
            float radius= vMag * 0.25f;
 
            tVec3 centerToP1;
            Vec_Scale(&centerToP1, v, -0.25f);
                         
            // interpolate across two semicircles
            Vec_Subtract(&pArc1->m_center, p1, centerToP1);
            pArc1->m_radius= radius;
            pArc1->m_axis1= centerToP1;
            Vec_Scale(&pArc1->m_axis2, perpAxis, radius*invVMagSqr);
            pArc1->m_angle= c_Pi;
            pArc1->m_arcLen= c_Pi * radius;
                     
            Vec_Add(&pArc2->m_center, p2, centerToP1);
            pArc2->m_radius= radius;
            Vec_Scale(&pArc2->m_axis1, centerToP1, -1.0f);
            Vec_Scale(&pArc2->m_axis2, perpAxis, -radius*invVMagSqr);
            pArc2->m_angle= c_Pi;
            pArc2->m_arcLen= c_Pi * radius;
 
            return;
        }
        else
        {
            // compute distance value for equal tangents
            d = vDotV / (4.0f * vDotT2);
        }           
    }
    else
    {
        // use the positive result of the quadratic formula
        const float discriminant = vDotT*vDotT + denominator*vDotV;
        d = (-vDotT + sqrt(discriminant)) / denominator;
    }
 
    // compute the connection point (i.e. the mid point)
    tVec3 pm;
    Vec_Subtract(&pm, t1, t2);
    Vec_AddScaled(&pm, p2, pm, d);
    Vec_Add(&pm, pm, p1);
    Vec_Scale(&pm, pm, 0.5f);
 
    // compute vectors from the end points to the mid point
    tVec3 p1ToPm, p2ToPm;
    Vec_Subtract(&p1ToPm, pm, p1);
    Vec_Subtract(&p2ToPm, pm, p2);
                             
    // compute the arcs
    tVec3 center1, center2;
    float radius1, radius2;
    float angle1, angle2;
    BiarcInterp_ComputeArc( &center1, &radius1, &angle1, p1, t1, p1ToPm );
    BiarcInterp_ComputeArc( &center2, &radius2, &angle2, p2, t2, p2ToPm );
             
    // use the longer path around the circle if d is negative
    if (d < 0.0f)
    {
        angle1= Sign(angle1)*c_2Pi - angle1;
        angle2= Sign(angle2)*c_2Pi - angle2;
    }
 
    // output the arcs
    // (the radius will be set to zero when the arc is a straight line)
    pArc1->m_center = center1;
    pArc1->m_radius = radius1;
    Vec_Subtract(&pArc1->m_axis1, p1, center1); // redundant from Vec_BiarcInterp_ComputeArc
    Vec_Scale(&pArc1->m_axis2, t1, radius1);
    pArc1->m_angle = angle1;
    pArc1->m_arcLen = (radius1 == 0.0f) ? Vec_Magnitude(p1ToPm) : fabs(radius1 * angle1);
 
    pArc2->m_center = center2;
    pArc2->m_radius = radius2;
    Vec_Subtract(&pArc2->m_axis1, p2, center2); // redundant from Vec_BiarcInterp_ComputeArc
    Vec_Scale(&pArc2->m_axis2, t2, -radius2);
    pArc2->m_angle = angle2;
    pArc2->m_arcLen = (radius2 == 0.0f) ? Vec_Magnitude(p2ToPm) : fabs(radius2 * angle2);
}


void BiarcInterp
(
    tVec3 *                     pResult,    // interpolated point
    const tBiarcInterp_Arc &    arc1,
    const tBiarcInterp_Arc &    arc2,
    float                       frac        // [0,1] fraction along the biarc
)
{
    assert( frac >= 0.0f && frac <= 1.0f );
                         
    const float epsilon = 0.0001f;
                         
    // compute distance along biarc
    const float totalDist = arc1.m_arcLen + arc2.m_arcLen;
    const float fracDist = frac * totalDist;
                     
    // choose the arc to evaluate
    if (fracDist < arc1.m_arcLen)
    {
        if (arc1.m_arcLen < epsilon)
        {
            // just output the end point
            Vec_Add(pResult, arc1.m_center, arc1.m_axis1);
        }
        else
        {
            const float arcFrac = fracDist / arc1.m_arcLen;
            if (arc1.m_radius == 0.0f)
            {
                // interpolate along the line
                Vec_AddScaled(pResult, arc1.m_center, arc1.m_axis1, -arcFrac*2.0f + 1.0f);
            }
            else
            {
                // interpolate along the arc
                float angle = arc1.m_angle*arcFrac;
                float sinRot = sinf(angle);
                float cosRot = cosf(angle);
 
                Vec_AddScaled(pResult, arc1.m_center, arc1.m_axis1, cosRot);
                Vec_AddScaled(pResult, *pResult, arc1.m_axis2, sinRot);
            }
        }
    }
    else
    {
        if (arc2.m_arcLen < epsilon)
        {
            // just output the end point
            Vec_Add(pResult, arc1.m_center, arc1.m_axis2);
        }
        else
        {
            const float arcFrac = (fracDist-arc1.m_arcLen) / arc2.m_arcLen;
            if (arc2.m_radius == 0.0f)
            {
                // interpolate along the line
                Vec_AddScaled(pResult, arc2.m_center, arc2.m_axis1, arcFrac*2.0f - 1.0f);
            }
            else
            {
                // interpolate along the arc
                float angle = arc2.m_angle*(1.0f-arcFrac);
                float sinRot = sinf(angle);
                float cosRot = cosf(angle);
 
                Vec_AddScaled(pResult, arc2.m_center, arc2.m_axis1, cosRot);
                Vec_AddScaled(pResult, *pResult, arc2.m_axis2, sinRot);
            }
        }
    }
}

void generateArcPath(Motor *motor,Location &location,cv::Point2d dest_loc,cv::Point2d dest_dir,float speed){
    
    // cosntants
    const float pi = 3.141592627;

    assert(speed>=-1 and speed <= 1); // other normalizations are checked when arcs are computed
    cv::Point2d start_loc = location.get_location().first;
    cv::Point2d start_dir = location.get_location().second;
    //cv::Point2d start_loc = cv::Point2d(50,50);
    //cv::Point2d start_dir = cv::Point2d(0.707,0.707);

    // rotating + tranforming to machine coord. system
    float start_dir_angle = atan2(start_dir.y,start_dir.x);
    float dest_dir_angle = atan2(dest_dir.y,dest_dir.x);
    float diff_angle = pi/2 - start_dir_angle;
    cv::Point2d start_to_dest = dest_loc - start_loc; 
    float dist = sqrt( (start_to_dest.x)*(start_to_dest.x) +
        (start_to_dest.y)*(start_to_dest.y) ); 
    float dist_angle = atan2(start_to_dest.y,start_to_dest.x)+diff_angle;
    tVec3 start = {0,0,0} , end = {std::cos(dist_angle)*dist,std::sin(dist_angle)*dist,0};
    tVec3 start_v = {0,1,0} , end_v = {std::cos(diff_angle+dest_dir_angle),std::sin(diff_angle+dest_dir_angle),0};
    
    // variables used in biarc computation
    bool is_l_not_r;
    float center_radius1,center_radius2;
    tBiarcInterp_Arc pArc1,pArc2;
    BiarcInterp_ComputeArcs(&pArc1,&pArc2,start,start_v,end,end_v);

    // first arc
    center_radius1 = 0-pArc1.m_center.m_x;
    is_l_not_r = center_radius1 > 0;
    motor->go(center_radius1,speed);
    
    // second arc
    diff_angle = is_l_not_r? pArc1.m_angle : - pArc1.m_angle;
    float angle = pi/2 + diff_angle;
    
    // inter point
    tVec3 inter_point;
    BiarcInterp(&inter_point,pArc1,pArc2,pArc1.m_arcLen/(pArc1.m_arcLen+pArc2.m_arcLen));
    tVec3 dir = {std::cos(angle),std::sin(angle),0}, 
          ptoc = {pArc2.m_center.m_x-inter_point.m_x,pArc2.m_center.m_y-inter_point.m_y,0};
    tVec3 res;
    Vec_CrossProduct(&res,dir,ptoc);
    is_l_not_r = res.m_z > 0;// clockwise or counterclock
    center_radius2 = is_l_not_r? pArc2.m_radius:-pArc2.m_radius;
    
    // processing first arc
    float dist_to_middle = (start_loc.x-inter_point.m_x)*(start_loc.x-inter_point.m_x) 
                        +   (start_loc.y-inter_point.m_y)*(start_loc.y-inter_point.m_y);    
    motor->go(center_radius1, 0.6);
    while (1) {
        start_loc = location.get_location().first;
        start_dir = location.get_location().second;
        float temp_dist  = (start_loc.x-inter_point.m_x)*(start_loc.x-inter_point.m_x) 
                        +   (start_loc.y-inter_point.m_y)*(start_loc.y-inter_point.m_y);
        if(temp_dist > dist_to_middle){
            break;
        }dist_to_middle = temp_dist;
    }

    // processing second arc
    float dist_to_end = (start_loc.x - dest_loc.x)*(start_loc.x - dest_loc.x) 
                        +   (start_loc.y - dest_loc.y)*(start_loc.y - dest_loc.y);
    motor->go(center_radius2, 0.6);
    while (1) {
        start_loc = location.get_location().first;
        start_dir = location.get_location().second;
        float temp_dist  = (start_loc.x - dest_loc.x)*(start_loc.x - dest_loc.x) 
                        +   (start_loc.y - dest_loc.y)*(start_loc.y - dest_loc.y);
        if(temp_dist > dist_to_end){
            break;
        }dist_to_end = temp_dist;
    }
    motor->stop();
    return;
}
