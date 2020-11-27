#include <cmath>
#include <Util/exceptions.h>
#include "camera.h"
#include "shape.h"

using namespace Ray;
using namespace Util;

////////////
// Camera //
////////////

Ray3D Camera::getRay( int i , int j , int width , int height ) const
{
	double d = 10;
	double halfHeight = d * tan(this->heightAngle / 2);
	double halfWidth = halfHeight * width / height;

	Point3D center = this->position + d * this->forward;
	Point3D doubleHeightVector = this->up * halfHeight;
	Point3D doubleWidthVector = this->right * halfWidth;

	Point3D endpoint = Point3D(center) - doubleHeightVector - doubleWidthVector;
	endpoint = endpoint + ((j + 0.5) / height) * (doubleHeightVector * 2);
	endpoint = endpoint + ((i + 0.5) / width) * (doubleWidthVector * 2);

	Point3D direction = (endpoint - this->position).normalize();
	return Ray3D(this->position, direction);
}

void Camera::getRay_AA(int i, int j, int width, int height, std::vector<Ray3D>& result) const
{
	double d = 10;
	double halfHeight = d * tan(this->heightAngle / 2);
	double halfWidth = halfHeight * width / height;

	Point3D center = this->position + d * this->forward;
	Point3D doubleHeightVector = this->up * halfHeight;
	Point3D doubleWidthVector = this->right * halfWidth;

	Point3D endpointCenter = Point3D(center) - doubleHeightVector - doubleWidthVector;
	endpointCenter = endpointCenter + ((j + 0.5) / height) * (doubleHeightVector * 2);
	endpointCenter = endpointCenter + ((i + 0.5) / width) * (doubleWidthVector * 2);

	Point3D endpoint1 = endpointCenter - (static_cast<double>(rand()) / RAND_MAX / 2) / height * (doubleHeightVector * 2)
		- (static_cast<double>(rand()) / RAND_MAX / 2) / width * (doubleWidthVector * 2);
	Point3D endpoint2 = endpointCenter - (static_cast<double>(rand()) / RAND_MAX / 2) / height * (doubleHeightVector * 2)
		+ (static_cast<double>(rand()) / RAND_MAX / 2) / width * (doubleWidthVector * 2);
	Point3D endpoint3 = endpointCenter + (static_cast<double>(rand()) / RAND_MAX / 2) / height * (doubleHeightVector * 2)
		- (static_cast<double>(rand()) / RAND_MAX / 2) / width * (doubleWidthVector * 2);
	Point3D endpoint4 = endpointCenter + (static_cast<double>(rand()) / RAND_MAX / 2) / height * (doubleHeightVector * 2)
		+ (static_cast<double>(rand()) / RAND_MAX / 2) / width * (doubleWidthVector * 2);

	Point3D direction1 = (endpoint1 - this->position).normalize();
	Point3D direction2 = (endpoint2 - this->position).normalize();
	Point3D direction3 = (endpoint3 - this->position).normalize();
	Point3D direction4 = (endpoint4 - this->position).normalize();
	
	result.push_back(Ray3D(this->position, direction1));
	result.push_back(Ray3D(this->position, direction2));
	result.push_back(Ray3D(this->position, direction3));
	result.push_back(Ray3D(this->position, direction4));
}

void Camera::drawOpenGL( void ) const
{
	Point3D center = this->position + this->forward;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(this->position[0], this->position[1], this->position[2], center[0], center[1], center[2], this->up[0], this->up[1], this->up[2]);

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

void Camera::rotateUp( Point3D center , float angle )
{
	double theta = angle;
	Point3D v = this->position - center;
	Point3D axis = this->up;
	v = v * cos(theta) + Point3D::CrossProduct(axis, v) * sin(theta) + axis * axis.dot(v) * (1 - cos(theta));
	this->position = center + v;
	this->forward = this->forward * cos(theta) + Point3D::CrossProduct(axis, this->forward) * sin(theta) + axis * axis.dot(this->forward) * (1 - cos(theta));
	this->right = this->right * cos(theta) + Point3D::CrossProduct(axis, this->right) * sin(theta) + axis * axis.dot(this->right) * (1 - cos(theta));
}

void Camera::rotateRight( Point3D center , float angle )
{
	double theta = angle;
	Point3D v = this->position - center;
	Point3D axis = this->right;
	v = v * cos(theta) + Point3D::CrossProduct(axis, v) * sin(theta) + axis * axis.dot(v) * (1 - cos(theta));
	this->position = center + v;
	this->forward = this->forward * cos(theta) + Point3D::CrossProduct(axis, this->forward) * sin(theta) + axis * axis.dot(this->forward) * (1 - cos(theta));
	this->up = this->up * cos(theta) + Point3D::CrossProduct(axis, this->up) * sin(theta) + axis * axis.dot(this->up) * (1 - cos(theta));
}

void Camera::moveForward( float dist )
{
	this->position = this->position + this->forward * dist;
}

void Camera::moveRight( float dist )
{
	this->position = this->position + this->right * dist;
}

void Camera::moveUp( float dist )
{
	this->position = this->position + this->up * dist;
}
