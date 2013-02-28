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
	double t, abc_len, ray_dir_len, plane_nX, plane_nY, plane_nZ, ray_plane_n_angle, abc_dot_ray;
	t = (params[0]*ray.origin[0] + params[1]*ray.origin[1] + params[2]*ray.origin[2] + params[3])
		/ -(params[0]*ray.direction[0] + params[1]*ray.direction[1] + params[2]*ray.direction[2]);

	// No intersection or it is outside the view
	if (t < 0 || t > 1)
		return false;

	*depth = t;
	*posX = ray.origin[0] + t*ray.direction[0];
	*posY = ray.origin[1] + t*ray.direction[1];
	*posZ = ray.origin[2] + t*ray.direction[2];

	// Calculate angle between ray direction and plane normal
	abc_len = sqrt(pow(params[0],2) + pow(params[1],2) + pow(params[2],2));
	ray_dir_len = sqrt(pow(ray.origin[0],2) + pow(ray.origin[0],2) + pow(ray.origin[2],2));
	abc_dot_ray =  params[0]*ray.origin[0] + params[1]*ray.origin[1] + params[2]*ray.origin[2];
	ray_plane_n_angle = acos(abc_dot_ray/(abc_len*ray_dir_len));

	// Normal is the plane normal if angle > 90 degree
	*normalX = (ray_plane_n_angle > PI/2) ? params[0]/abc_len : -params[0]/abc_len;
	*normalY = (ray_plane_n_angle > PI/2) ? params[1]/abc_len : -params[1]/abc_len;
	*normalZ = (ray_plane_n_angle > PI/2) ? params[2]/abc_len : -params[2]/abc_len;


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
	double a,b,c,discrm, sqrt_discrm, t1, t2, t, center2intX, center2intY, center2intZ, center2intLen;

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
	if ((t1 < 0 || t1 >1) && (t2 < 0 || t2 > 1))
		return false;

	t = (t1 <= t2 && t1 >= 0) ? t1 : t2;
	
	*depth = t;
	*posX = ray.origin[0] + t*ray.direction[0];
	*posY = ray.origin[1] + t*ray.direction[1];
	*posZ = ray.origin[2] + t*ray.direction[2];

	// Normal is <intersection-center> normalized
	center2intX = *posX - center[0];
	center2intY = *posY - center[1];
	center2intZ = *posZ - center[2];
	center2intLen = sqrt(pow(center2intX,2) + pow(center2intY,2) + pow(center2intZ,2));
	*normalX = center2intX/center2intLen;
	*normalY = center2intY/center2intLen;
	*normalZ = center2intZ/center2intLen;

	//////////*********** END OF CODE TO CHANGE *******///k/////////

	return true;
}

///////////////////////////////////////////////////////////////////////////
