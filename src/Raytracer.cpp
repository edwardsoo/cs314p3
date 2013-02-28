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

#include <Raytracer.h>
#include <Operations.h>

#include <stdio.h>
#include <stdlib.h>
#include <float.h>

#include <iostream>
using namespace std;

///////////////////////////////////////////////////////////////////////////

void	
	Raytracer::raytraceScene(	const char *filename,
	const char *depth_filename,
	std::vector<PointLight> *lights, 
	std::vector<Plane> *planes,	
	std::vector<Sphere> *spheres,
	Camera *camera, int *resolution	)
{

	// pixel color buffer
	unsigned char *colorbuffer = new unsigned char[resolution[0]*resolution[1]*3];
	for(int i=0; i<3*resolution[0]*resolution[1]; i++) {
		colorbuffer[i] = (unsigned char)0;
	}
	// depth buffer
	double *depthbuffer = new double[resolution[0]*resolution[1]];
	for(int i=0; i<resolution[0]*resolution[1]; i++) {
		depthbuffer[i] = 1.0;
	}

	// the magical ray setup
	Vec3 f(	camera->center[0]-camera->position[0], 
		camera->center[1]-camera->position[1], 
		camera->center[2]-camera->position[2]);
	Vec3 up(camera->up[0],camera->up[1],camera->up[2]);
	// normalized up vector
	up.normalize();
	Vec3 right = f.cross(up);
	// normalized vector pointing to the side
	right.normalize();

	// step for each pixel
	double stepx = 2*tan(deg2rad(camera->fovy)*camera->aspect/2.0)*f.length() / resolution[0];
	double stepy = 2*tan(deg2rad(camera->fovy)/2.0)*f.length() / resolution[1];

	for(int y=0; y<resolution[1]; y++ ) {
		for(int x=0; x<resolution[0]; x++) {

			//////////////////////////////////////////////////////////////////////////////////
			//	the following lines implement a simple orthographic camera
			//	you have to modify the code for generating pespective rays
			//	i.e. all rays (theoretical) origin from the camera position and 
			//	propagate into the pixelLookAt
			//
			//	make sure that the ray's origin is not the camera center but 
			//	on the near plane. the direction of the ray should be a vector,
			//	that is computed as pointOnFarPlane-pointOnNearPlane, i.e. 
			//	adding it to the t parameter in the ray equation 
			//	point = pointOnNearPlane + t*rayDirection
			//	is always between 0 and 1. This will be the value in the depth
			//	buffer.
			//////////////////////////////////////////////////////////////////////////////////
			// generate the appropriate ray for this pixel

			//////////*********** START OF CODE TO CHANGE *******////////////

			int alpha = x-resolution[0]/2;
			int beta  = y-resolution[1]/2;
			Vec3 pixelLookAt(	camera->center[0] + alpha*stepx*right[0] + beta*stepy*up[0],
				camera->center[1] + alpha*stepx*right[1] + beta*stepy*up[1],
				camera->center[2] + alpha*stepx*right[2] + beta*stepy*up[2]);			


			Vec3 pixelStart(	camera->position[0] + alpha*stepx*right[0] + beta*stepy*up[0],
				camera->position[1] + alpha*stepx*right[1] + beta*stepy*up[1],
				camera->position[2] + alpha*stepx*right[2] + beta*stepy*up[2]);

			Vec3 rayDirection(	pixelLookAt[0]-pixelStart[0], 
				pixelLookAt[1]-pixelStart[1], 
				pixelLookAt[2]-pixelStart[2]);
			rayDirection.normalize();

			Ray pixelRay(	pixelStart[0] + camera->zNear*rayDirection[0], 
				pixelStart[1] + camera->zNear*rayDirection[1],
				pixelStart[2] + camera->zNear*rayDirection[2],
				(camera->zFar-camera->zNear)*rayDirection[0],
				(camera->zFar-camera->zNear)*rayDirection[1],
				(camera->zFar-camera->zNear)*rayDirection[2]	);

			//////////*********** END OF CODE TO CHANGE *******////////////

			//////////////////////////////////////////////////////////////////////////////////

			int currentRayRecursion=0;
			double red=0.0, green=0.0, blue=0.0, depth=1.0;

			// calculate the pixel value by shooting the ray into the scene
			traceRay(	pixelRay, lights, planes, spheres, camera, &currentRayRecursion, 
				&red ,&green, &blue, &depth);

			// depth test
			if( (depth<depthbuffer[x+y*resolution[0]]) && (depth>=0.0)) {

				// red
				colorbuffer[3*(x+y*resolution[0])]=(unsigned char)(255.0*red);
				// green
				colorbuffer[3*(x+y*resolution[0])+1]=(unsigned char)(255.0*green);
				// blue
				colorbuffer[3*(x+y*resolution[0])+2]=(unsigned char)(255.0*blue);

				// depth 
				depthbuffer[x+y*resolution[0]] = depth;
			}
		}
	}
	writeImage(filename, colorbuffer, resolution);

	unsigned char *depthbuffer_uc = new unsigned char[resolution[0]*resolution[1]];
	for(int i=0; i<resolution[0]*resolution[1]; i++) {
		depthbuffer_uc[i] = (unsigned char)(255.0*depthbuffer[i]);
	}
	writeImage(depth_filename, depthbuffer_uc, resolution, true);

	delete [] depthbuffer_uc;
	delete [] depthbuffer;
	delete [] colorbuffer;
}

/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//	this function shoots a ray into the scene and calculates
//	the closest point of intersection
//	
//	if the ray hits a surface that is reflecting the rays have to be recursively
//	shot into the scene starting from that point into the proper direction
//	(using Snell's law of reflection). you need to implement the ray reflection 
//	and color blending between the reflected color and the local color (i.e.
//	direct lighting and indirect lighting). weight the reflection term by
//	1-material.reflect
//
/////////////////////////////////////////////////////////////////////////////////
void	
	Raytracer::traceRay(	Ray pixelRay, 
	std::vector<PointLight> *lights, 
	std::vector<Plane> *planes,	
	std::vector<Sphere> *spheres, 
	Camera *camera,
	int *currRayRecursion,
	double *red , double *green, double *blue, double *depth )
{

	// did it intersect at all?
	bool bIntersect=false;
	// depth of the closest intersection
	double intersectDepth=*depth;
	// 3D position of the closest intersection
	double intersectPos[3];
	// normal at the point of closest intersection
	double intersectNormal[3];
	// the material of the object at the intersection point
	Material intersectMaterial;

	// calculate intersection of ray with all planes
	foreach(p, (*planes), vector<Plane>) {

		double iDepth, iPos[3], iNormal[3];

		if( p->intersect(	pixelRay, &iDepth, 
			&iPos[0], &iPos[1], &iPos[2],
			&iNormal[0], &iNormal[1], &iNormal[2]))
		{

			// depth test
			if(iDepth<intersectDepth) {
				bIntersect = true;
				intersectDepth = iDepth;
				intersectPos[0]=iPos[0]; intersectPos[1]=iPos[1]; intersectPos[2]=iPos[2];
				intersectNormal[0]=iNormal[0]; intersectNormal[1]=iNormal[1]; intersectNormal[2]=iNormal[2];
				intersectMaterial = p->material;
			}
		}		
	}

	// calculate intersection of ray with all spheres
	foreach(s, (*spheres), vector<Sphere>) {

		double iDepth, iPos[3], iNormal[3];

		if( s->intersect(	pixelRay, &iDepth, 
			&iPos[0], &iPos[1], &iPos[2],
			&iNormal[0], &iNormal[1], &iNormal[2]))
		{

			// depth test
			if(iDepth<intersectDepth) {
				bIntersect = true;
				intersectDepth = iDepth;
				intersectPos[0]=iPos[0]; intersectPos[1]=iPos[1]; intersectPos[2]=iPos[2];
				intersectNormal[0]=iNormal[0]; intersectNormal[1]=iNormal[1]; intersectNormal[2]=iNormal[2];
				intersectMaterial = s->material;
			}
		}
	}

	if(bIntersect) {

		*depth = intersectDepth;

		shade(	intersectPos[0], intersectPos[1], intersectPos[2],
			intersectNormal[0], intersectNormal[1], intersectNormal[2],
			intersectMaterial, camera, 
			lights, planes,	spheres,
			red, green, blue);

		/////////////////////////////////////////////////////////////////////////////////////
		// recurse for reflections
		if( (*currRayRecursion<MAX_RAY_RECURSION) && (intersectMaterial.reflect>0.0) ) {				
			(*currRayRecursion) += 1;

			//////////*********** START OF CODE TO CHANGE *******////////////

			//////////*********** END OF CODE TO CHANGE *******////////////

		}
		/////////////////////////////////////////////////////////////////////////////////////

		// clip colors between 0.0 and 1.0
		*red = (*red > 1.0) ? 1.0 : *red;
		*red = (*red < 0.0) ? 0.0 : *red;
		*green = (*green > 1.0) ? 1.0 : *green;
		*green = (*green < 0.0) ? 0.0 : *green;
		*blue = (*blue > 1.0) ? 1.0 : *blue;
		*blue = (*blue < 0.0) ? 0.0 : *blue;

	}
}

///////////////////////////////////////////////////////////////////////////

double getVectorLen(double x, double y, double z) 
{
	return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

void getVector(double toX, double toY, double toZ, double fromX, double fromY, double fromZ, 
			   double* x, double* y, double* z) 
{
	*x = toX - fromX;
	*y = toY - fromY;
	*z = toZ - fromZ;
}

double getDotProduct(double x1, double y1, double z1, double x2, double y2, double z2) {
	return x1*x2 + y1*y2 + z1*z2;
}

void getReflection(double normalX, double normalY, double normalZ, double p2lX, double p2lY, double p2lZ, 
				   double* reflectX, double* reflectY, double* reflectZ)
{
	double p2lDotNormal;
	p2lDotNormal = getDotProduct(normalX, normalY,normalZ, p2lX, p2lY, p2lZ);
	*reflectX = 2*p2lDotNormal*normalX - p2lX;
	*reflectY = 2*p2lDotNormal*normalY - p2lY;
	*reflectZ = 2*p2lDotNormal*normalZ - p2lZ;
}

/////////////////////////////////////////////////////////////////////////////////
//	here you need to calculate the proper shading of a given point at posX, posY,
//	posZ and corresponding normal normalX, normalY and normalZ
//	
//	the material of the point is given as material, the camera as camera
//	and all objects in the scenes as planes and spheres
//	
//	lights is a list of all the light source, make sure to calculate the 
//	lighting for each one of them and add the color values, the material
//	emission part is only added once!
// 
//	write the color values in red, green and blue
// 
//	you also have to add the code that calculates if the point is in the 
//	shadow for each light source and weight the computed color value
//	bu 1-material.shadow
/////////////////////////////////////////////////////////////////////////////////
void	
	Raytracer::shade(	double posX, double posY, double posZ,
	double normalX, double normalY, double normalZ,
	Material material, Camera *camera, 
	std::vector<PointLight> *lights, 
	std::vector<Plane>		*planes,	
	std::vector<Sphere>		*spheres,
	double *red, double *green, double *blue)
{
	// debug normal
	/**red = 1.0*normalX;
	*green = 1.0*normalY;
	*blue = 1.0*normalZ;
	return;*/

	double p2eX, p2eY, p2eZ, p2eLen, p2eNDotNormal;
	double p2pX, p2pY, p2pZ;

	// calculate emissive part here.
	getVector(camera->position[0], camera->position[1], camera->position[2], posX, posY, posZ, &p2eX, &p2eY, &p2eZ);
	p2eLen = getVectorLen(p2eX,p2eY,p2eZ);

	// (normalized point-to-camera) dot (normal)
	p2eNDotNormal = getDotProduct(p2eX/p2eLen, p2eY/p2eLen, p2eZ/p2eLen, normalX, normalY, normalZ);
	if (p2eNDotNormal < 0)
		p2eNDotNormal = 0;

	*red = material.emission[0]*p2eNDotNormal;
	*green = material.emission[1]*p2eNDotNormal;
	*blue = material.emission[2]*p2eNDotNormal;

	// Point-to-pixel distance
	p2pZ = p2eZ - camera->zNear;
	p2pX = p2eX*p2pZ/p2eZ;
	p2pY = p2eY*p2pZ/p2eZ;

	foreach(light, (*lights), vector<PointLight>) {

		//////////*********** START OF CODE TO CHANGE *******////////////
		// add color value of this light source to color
		// include ambient, diffuse and specular
		// make sure to add the light attenuation
		// effect for the diffuse and specular term
		double ambient[3], diffuse[3], specular[3];
		double p2lX, p2lY, p2lZ, p2lLen, reflectX, reflectY, reflectZ, reflectLen, p2lDotNormal, p2eDotReflect, p2eDotReflectToShiny;
		double hX, hY, hZ, hLen, hDotNormal, hDotNormalToShiny, l2pAttenuation;

		// Calculate point-to-light, reflection vectors and attenuation factor
		getVector(light->position[0], light->position[1], light->position[2], posX, posY, posZ, &p2lX, &p2lY, &p2lZ);
		p2lLen = getVectorLen(p2lX, p2lY, p2lZ);
		getReflection(normalX, normalY, normalZ, p2lX/p2lLen, p2lY/p2lLen, p2lZ/p2lLen, &reflectX, &reflectY, &reflectZ);
		reflectLen = getVectorLen(reflectX, reflectY, reflectZ);
		l2pAttenuation = 1/(light->attenuation[0]+light->attenuation[1]*p2lLen+light->attenuation[2]*pow(p2lLen,2));

		// Ambient
		ambient[0] = light->ambient[0]*material.ambient[0];
		ambient[1] = light->ambient[1]*material.ambient[1];
		ambient[2] = light->ambient[2]*material.ambient[2];

		// Diffuse
		p2lDotNormal = getDotProduct(p2lX/p2lLen, p2lY/p2lLen, p2lZ/p2lLen, normalX, normalY,normalZ);
		if (p2lDotNormal > 0) {
			diffuse[0] = light->diffuse[0]*material.diffuse[0]*p2lDotNormal*l2pAttenuation;
			diffuse[1] = light->diffuse[1]*material.diffuse[1]*p2lDotNormal*l2pAttenuation;
			diffuse[2] = light->diffuse[2]*material.diffuse[2]*p2lDotNormal*l2pAttenuation;
		} else {
			diffuse[0] = diffuse[1] = diffuse[2] = 0;
		}


		// Specular
		p2eDotReflect = getDotProduct(reflectX/reflectLen, reflectY/reflectLen, reflectZ/reflectLen, 
			p2eX/p2eLen, p2eY/p2eLen, p2eZ/p2eLen);
		if (p2eDotReflect > 0) {
			p2eDotReflectToShiny = pow(p2eDotReflect, material.shininess);
			specular[0] = light->specular[0]*material.specular[0]*p2eDotReflectToShiny*l2pAttenuation;
			specular[1] = light->specular[1]*material.specular[1]*p2eDotReflectToShiny*l2pAttenuation;
			specular[2] = light->specular[2]*material.specular[2]*p2eDotReflectToShiny*l2pAttenuation;
		} else {
			specular[0] = specular[1] = specular[2] = 0;
		}

		*red += ambient[0] + diffuse[0] + specular[0];
		*green += ambient[1] + diffuse[1] + specular[1];
		*blue += ambient[2] + diffuse[2] + specular[2];

		// Blinn-Phong Model
		// Halfway vector h
		/*hX = (p2lX + p2eX)/2;
		hY = (p2lY + p2eY)/2;
		hZ = (p2lZ + p2eZ)/2;
		hLen = getVectorLen(hX, hY, hZ);
		hDotNormal = getDotProduct(hX/hLen, hY/hLen, hZ/hLen, normalX, normalY, normalZ);
		if (hDotNormal > 0) {
			hDotNormalToShiny = pow(hDotNormal, material.shininess);
		} else {
		}*/



		/////////////////////////////////////////////////////////////////////////////////////
		// shoot a ray to every light source to see if the point is in shadow

		//////////*********** END OF CODE TO CHANGE *******////////////

		if(material.shadow != 0.0) {

			//////////*********** START OF CODE TO CHANGE *******////////////

			// add your code for shadows here

			//////////*********** END OF CODE TO CHANGE *******////////////
		}

		// end of shadows
		/////////////////////////////////////////////////////////////////////////////////////

		//////////*********** START OF CODE TO CHANGE *******////////////

		// add calculated color to final color

		//////////*********** END OF CODE TO CHANGE *******////////////
	}	
}

///////////////////////////////////////////////////////////////////////////

bool	
	Raytracer::writeImage(	const char *filename, 
	unsigned char *imageBuffer, 
	int *resolution, 
	bool greyscale) {

		FILE *fp = fopen(filename,"wb");
		if (!fp) {
			printf("Unable to open file '%s'\n",filename);
			return false;
		}

		const int maxVal=255;

		fprintf(fp, "P6 ");
		fprintf(fp, "%d %d ", resolution[0], resolution[1]);
		fprintf(fp, "%d", maxVal);
		putc(13,fp);

		int numChannels=3;
		if(greyscale)
			numChannels=1;

		for(int y=resolution[1]-1; y>=0; y-- ) {
			for(int x=0; x<resolution[0]; x++) {
				// red or depth
				putc(imageBuffer[numChannels*(x+y*resolution[0])],fp);
				if(!greyscale) {
					// green
					putc(imageBuffer[numChannels*(x+y*resolution[0])+1],fp);
					// blue
					putc(imageBuffer[numChannels*(x+y*resolution[0])+2],fp);
				} else {
					// depth
					putc(imageBuffer[numChannels*(x+y*resolution[0])],fp);
					// depth
					putc(imageBuffer[numChannels*(x+y*resolution[0])],fp);
				}
			}
		}
		fclose(fp);
		return true;
}

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
