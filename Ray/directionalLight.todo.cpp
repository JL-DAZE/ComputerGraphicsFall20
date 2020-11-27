#include <cmath>
#include <Util/exceptions.h>
#include "directionalLight.h"
#include "scene.h"

using namespace Ray;
using namespace Util;

//////////////////////
// DirectionalLight //
//////////////////////

Point3D DirectionalLight::getAmbient( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	return (this->_ambient * iInfo.material->ambient);
}

Point3D DirectionalLight::getDiffuse( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	Point3D light_dir = this->_direction * -1;
	double normalLightAngle = iInfo.normal.dot(light_dir);
	normalLightAngle = (normalLightAngle < 0) ? 0 : normalLightAngle;

	return (this->_diffuse * iInfo.material->diffuse * normalLightAngle);
}

Point3D DirectionalLight::getSpecular( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	Point3D viewerDirection = -ray.direction;
	Point3D lightDirection = -this->_direction;
	Point3D reflectDirection = 2 * (iInfo.normal.dot(lightDirection)) * iInfo.normal - lightDirection;
	double viewerReflectAngle = viewerDirection.dot(reflectDirection);
	viewerReflectAngle = (viewerReflectAngle < 0) ? 0 : viewerReflectAngle;

	return (iInfo.material->specular * pow(viewerReflectAngle, iInfo.material->specularFallOff) * this->_specular);
}

bool DirectionalLight::isInShadow( const RayShapeIntersectionInfo& iInfo , const Shape* shape ) const
{
	Ray3D r = Ray3D();
	r.position = iInfo.position;
	r.direction = this->_direction;

	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	BoundingBox1D BBox = BoundingBox1D(Point1D(-Infinity), Point1D(Infinity));
	shape->intersect(r, tempIInfo, BBox);

	Point3D error = Point3D(1e-9, 1e-9, 1e-9);
	BoundingBox3D errorBox = BoundingBox3D(error * -1, error);
	return !(errorBox.isInside(iInfo.position - tempIInfo.position));
}

Point3D DirectionalLight::transparency( const RayShapeIntersectionInfo &iInfo , const Shape &shape , Point3D cLimit ) const
{
	Ray3D r = Ray3D();
	r.direction = this->_direction * -1;
	r.position = iInfo.position + (r.direction * Epsilon);

	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	BoundingBox1D bBox = BoundingBox1D(Point1D(Epsilon), Point1D(Infinity));
	double t = shape.intersect(r, tempIInfo, bBox);
	if (t < Infinity)
	{
		Point3D localTransparency = tempIInfo.material->transparent;
		if (localTransparency[0] < cLimit[0] && localTransparency[1] < cLimit[1] && localTransparency[2] < cLimit[2])
			return localTransparency;
		else
		{
			Point3D subsequentTransparency = this->transparency(tempIInfo, shape, cLimit / localTransparency);
			return (localTransparency * subsequentTransparency);
		}
	}
	else
		return Point3D(1.0, 1.0, 1.0);
}

void DirectionalLight::drawOpenGL( int index , GLSLProgram * glslProgram ) const
{
	float position[4] = { this->_direction[0], this->_direction[1], this->_direction[2], 0.0 };
	float ambient[4] = { this->_ambient[0], this->_ambient[1], this->_ambient[2], 1.0 };
	float diffuse[4] = { this->_diffuse[0], this->_diffuse[1], this->_diffuse[2], 1.0 };
	float specular[4] = { this->_specular[0], this->_specular[1], this->_specular[2], 1.0 };

	glLightfv(GL_LIGHT0 + index, GL_POSITION, position);
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, specular);

	glEnable(GL_LIGHT0 + index);

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}
