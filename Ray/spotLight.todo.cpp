#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "spotLight.h"

using namespace Ray;
using namespace Util;

///////////////
// SpotLight //
///////////////
Point3D SpotLight::getAmbient( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	return (this->_ambient * iInfo.material->ambient);
}

Point3D SpotLight::getDiffuse( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	Point3D lightDirection = (this->_location - iInfo.position).normalize();
	double lightObjectAngle = this->_direction.dot(lightDirection * -1);
	if (lightObjectAngle <= cos(this->_cutOffAngle))
	{
		return Point3D();
	}
	else
	{
		double dist = (this->_location - iInfo.position).magnitude();
		double attenuation = this->_constAtten + this->_linearAtten * dist + this->_quadAtten * dist * dist;
		Point3D I_L = (this->_diffuse * pow(lightObjectAngle, this->_dropOffRate)) / attenuation;
		double normalLightAngle = iInfo.normal.dot(lightDirection);
		normalLightAngle = (normalLightAngle < 0) ? 0 : normalLightAngle;

		return (iInfo.material->diffuse * I_L * normalLightAngle);
	}
}

Point3D SpotLight::getSpecular( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	Point3D lightDirection = (this->_location - iInfo.position).normalize();
	double lightObjectAngle = this->_direction.dot(lightDirection * -1);
	if (lightObjectAngle <= cos(this->_cutOffAngle))
	{
		return Point3D();
	}
	else
	{
		double dist = (this->_location - iInfo.position).magnitude();
		double attenuation = this->_constAtten + this->_linearAtten * dist + this->_quadAtten * dist * dist;
		Point3D I_L = this->_diffuse * pow(lightObjectAngle, this->_dropOffRate) / attenuation;
		Point3D viewerDirection = -ray.direction;
		Point3D lightDirection = (this->_location - iInfo.position).normalize();
		Point3D reflectDirection = 2 * (iInfo.normal.dot(lightDirection)) * iInfo.normal - lightDirection;
		double viewerReflectAngle = viewerDirection.dot(reflectDirection);
		viewerReflectAngle = (viewerReflectAngle < 0) ? 0 : viewerReflectAngle;

		return (iInfo.material->specular * pow(viewerReflectAngle, iInfo.material->specularFallOff) * I_L);
	}
}

bool SpotLight::isInShadow( const RayShapeIntersectionInfo& iInfo , const Shape* shape ) const
{
	Ray3D r = Ray3D();
	r.position = this->_location;
	r.direction = (iInfo.position - this->_location).normalize();
	double t = (iInfo.position - this->_location).magnitude();

	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	BoundingBox1D BBox = BoundingBox1D(Point1D(Epsilon), Point1D(t));
	shape->intersect(r, tempIInfo, BBox);

	Point3D error = Point3D(1e-9, 1e-9, 1e-9);
	BoundingBox3D errorBox = BoundingBox3D(error * -1, error);
	return !(errorBox.isInside(iInfo.position - tempIInfo.position));
}

Point3D SpotLight::transparency( const RayShapeIntersectionInfo &iInfo , const Shape &shape , Point3D cLimit ) const
{
	Ray3D r = Ray3D();
	r.direction = (this->_location - iInfo.position).normalize();
	r.position = iInfo.position + (r.direction * Epsilon);

	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	double tLimit = (this->_location - iInfo.position).magnitude();
	BoundingBox1D bBox = BoundingBox1D(Point1D(Epsilon), Point1D(tLimit));
	double t = shape.intersect(r, tempIInfo, bBox);
	if (t < Infinity)
	{
		Point3D localTransparency = tempIInfo.material->transparent;
		if (localTransparency[0] < cLimit[0] && localTransparency[1] < cLimit[1] && localTransparency[2] < cLimit[2])
			return Point3D(0.0, 0.0, 0.0);
		else
		{
			Point3D subsequentTransparency = this->transparency(tempIInfo, shape, cLimit / localTransparency);
			return (localTransparency * subsequentTransparency);
		}
	}
	else
		return Point3D(1.0, 1.0, 1.0);
}

void SpotLight::drawOpenGL( int index , GLSLProgram * glslProgram ) const
{
	float position[4] = { this->_location[0], this->_location[1], this->_location[2], 1.0 };
	float ambient[4] = { this->_ambient[0], this->_ambient[1], this->_ambient[2], 1.0 };
	float diffuse[4] = { this->_diffuse[0], this->_diffuse[1], this->_diffuse[2], 1.0 };
	float specular[4] = { this->_specular[0], this->_specular[1], this->_specular[2], 1.0 };
	float direction[3] = { this->_direction[0], this->_direction[1], this->_direction[2] };
	float constAtten[1] = { this->_constAtten };
	float linearAtten[1] = { this->_linearAtten };
	float quadAtten[1] = { this->_quadAtten };
	float cutoff[1] = { this->_cutOffAngle / (Pi / 2) * 90 };
	float dropoff[1] = { this->_dropOffRate };

	glLightfv(GL_LIGHT0 + index, GL_POSITION, position);
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_DIRECTION, direction);
	glLightfv(GL_LIGHT0 + index, GL_CONSTANT_ATTENUATION, constAtten);
	glLightfv(GL_LIGHT0 + index, GL_LINEAR_ATTENUATION, linearAtten);
	glLightfv(GL_LIGHT0 + index, GL_QUADRATIC_ATTENUATION, quadAtten);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_CUTOFF, cutoff);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_EXPONENT, dropoff);

	glEnable(GL_LIGHT0 + index);

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}
