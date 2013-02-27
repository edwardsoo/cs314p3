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

	//////////*********** END OF CODE TO CHANGE *******////////////

	return true;
}

///////////////////////////////////////////////////////////////////////////
