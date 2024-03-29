///////////////////////////////////////////////////////////////////////////
//
//	simple raytracer
//
//	UBC CS 314 - Feburary 2013
//	instructor: Tamara Munzner [tmm@cs.ubc.ca] 
//	assignment 3
//
//	originally written by Gordon Wetzstein
//	updated by and questions should go to: Yufeng Zhu [mike323@cs.ubc.ca]
//	
///////////////////////////////////////////////////////////////////////////

#include <Primitives.h>

#include <math.h>

///////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//	calculate the intersection of a plane the given ray
//	the ray has an origin and a direction, ray = origin + t*direction
//	find the t parameter, return true if it is between 0.0 and 1.0, false 
//	otherwise, write the results in following variables:
//	depth	- t \in [0.0 1.0]
//	posX	- x position of intersection point, nothing if no intersection
//	posY	- y position of intersection point, nothing if no intersection
//	posZ	- z position of intersection point, nothing if no intersection
//	normalX	- x component of normal at intersection point, nothing if no intersection
//	normalX	- y component of normal at intersection point, nothing if no intersection
//	normalX	- z component of normal at intersection point, nothing if no intersection
//
/////////////////////////////////////////////////////////////////////////////////
bool 
	Plane::intersect(Ray ray, double *depth,
	double *posX, double *posY, double *posZ,
	double *normalX, double *normalY, double *normalZ)

{
	//////////*********** START OF CODE TO CHANGE *******////////////
	/* 
	* Ray-plane intersection implicit equation:
	* A(ray_o_x + t*ray_d_x) + B(ray_o_y + t*ray_d_y) + 
	* C(ray_o_z + t*ray_d_z) + D = 0
	*/
	Vec3 planeNormal(params[0], params[1], params[2]);
	planeNormal.normalize();
	Vec3 rayDirection(ray.direction[0], ray.direction[1], ray.direction[2]);
	Vec3 rayOrigin(ray.origin[0], ray.origin[1], ray.origin[2]);

	double t, planeNormalRayAngle, planeNormalDotRayDirection;
	t = - (planeNormal.dot(rayOrigin) + params[3])
		/ planeNormal.dot(rayDirection);

	// No intersection or it is outside the view
	if (t != t || !(t >= 0 && t <= 1))
		return false;

	*depth = t;
	*posX = ray.origin[0] + t*ray.direction[0];
	*posY = ray.origin[1] + t*ray.direction[1];
	*posZ = ray.origin[2] + t*ray.direction[2];

	// Calculate angle between ray direction and plane normal
	planeNormal.normalize();
	rayDirection.normalize();
	planeNormalDotRayDirection = planeNormal.dot(rayDirection);
	planeNormalRayAngle = acos(planeNormalDotRayDirection);

	// Normal is the plane normal if angle > 90 degree
	*normalX = (planeNormalRayAngle > PI/2) ? planeNormal[0] : -planeNormal[0];
	*normalY = (planeNormalRayAngle > PI/2) ? planeNormal[1] : -planeNormal[1];
	*normalZ = (planeNormalRayAngle > PI/2) ? planeNormal[2] : -planeNormal[2];

	//////////*********** END OF CODE TO CHANGE *******////////////

	return true;
}

///////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//	calculate the intersection of a sphere the given ray
//	the ray has an origin and a direction, ray = origin + t*direction
//	find the t parameter, return true if it is between 0.0 and 1.0, false 
//	otherwise, write the results in following variables:
//	depth	- t \in [0.0 1.0]
//	posX	- x position of intersection point, nothing if no intersection
//	posY	- y position of intersection point, nothing if no intersection
//	posZ	- z position of intersection point, nothing if no intersection
//	normalX	- x component of normal at intersection point, nothing if no intersection
//	normalX	- y component of normal at intersection point, nothing if no intersection
//	normalX	- z component of normal at intersection point, nothing if no intersection
//
//	attention: a sphere has usually two intersection points make sure to return 
//	the one that is closest to the ray's origin and still in the viewing frustum
//
/////////////////////////////////////////////////////////////////////////////////
bool 
	Sphere::intersect(Ray ray, double *depth,	
	double *posX, double *posY, double *posZ,
	double *normalX, double *normalY, double *normalZ)

{
	//////////*********** START OF CODE TO CHANGE *******////////////
	/* 
	* Ray-sphere intersection implicit equation:
	* (ray_o_x + t*ray_d_x - sph_c_x)^2 + (ray_o_y + t*ray_d_y - sph_c_y)^2 + 
	* (ray_o_z + t*ray_d_z - sph_c_z)^2 = sph_rad^2
	*/
	double a,b,c,discrm, sqrt_discrm, t1, t2, t;

	a = pow(ray.direction[0],2) + pow(ray.direction[1],2) + pow(ray.direction[2],2);
	b = (ray.origin[0]*ray.direction[0] + ray.origin[1]*ray.direction[1] + ray.origin[2]*ray.direction[2]
	- ray.direction[0]*center[0] - ray.direction[1]*center[1] - ray.direction[2]*center[2]) * 2;
	c = pow(ray.origin[0],2) + pow(ray.origin[1],2) + pow(ray.origin[2],2)
		- (ray.origin[0]*center[0] + ray.origin[1]*center[1] + ray.origin[2]*center[2]) * 2
		+ pow(center[0],2) + pow(center[1],2) + pow(center[2],2) - pow(radius,2);

	discrm = pow(b,2) - 4*a*c;

	// No solution to quadratic equation
	if (discrm < 0)
		return false;

	sqrt_discrm = sqrt(discrm);
	t1 = (-b + sqrt_discrm)/(2*a);
	t2 = (-b - sqrt_discrm)/(2*a);

	// Intersections lie outside viewing frustum
	if ((t1 != t1 || !(t1 >= 0 && t1 <= 1)) &&
		(t2 != t2 || !(t2 >= 0 && t2 <= 1)))
		return false;

	t = (t1 <= t2 && t1 >= 0) ? t1 : t2;

	*depth = t;
	*posX = ray.origin[0] + t*ray.direction[0];
	*posY = ray.origin[1] + t*ray.direction[1];
	*posZ = ray.origin[2] + t*ray.direction[2];

	// Normal is <intersection-center> normalized
	Vec3 center2intersect(*posX - center[0], *posY - center[1], *posZ - center[2]);
	center2intersect.normalize();
	*normalX = center2intersect[0];
	*normalY = center2intersect[1];
	*normalZ = center2intersect[2];

	//////////*********** END OF CODE TO CHANGE *******///k/////////

	return true;
}

///////////////////////////////////////////////////////////////////////////
