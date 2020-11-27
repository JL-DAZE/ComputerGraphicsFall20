#include <cmath>
#include <Util/exceptions.h>
#include "triangle.h"
#include <vector>
#include <algorithm>

using namespace Ray;
using namespace Util;
using std::vector;

//////////////
// Triangle //
//////////////

void Triangle::init( const LocalSceneData &data )
{
	// Set the vertex pointers
	for( int i=0 ; i<3 ; i++ )
	{
		if( _vIndices[i]==-1 ) THROW( "negative vertex index: %d" , _vIndices[i] );
		else if( _vIndices[i]>=data.vertices.size() ) THROW( "vertex index out of bounds: %d <= %d" , _vIndices[i] , (int)data.vertices.size() );
		else _v[i] = &data.vertices[ _vIndices[i] ];
	}

	this->mPlane = Plane3D(this->_v[0]->position, this->_v[1]->position, this->_v[2]->position);

	this->side0 = this->_v[1]->position - this->_v[0]->position;
	this->side1 = this->_v[2]->position - this->_v[0]->position;

	this->d00 = this->side0.dot(this->side0);
	this->d01 = this->side0.dot(this->side1);
	this->d11 = this->side1.dot(this->side1);
	this->denominator = d00 * d11 - d01 * d01;
}

void Triangle::updateBoundingBox( void )
{
	BoundingBox3D result;
	result = BoundingBox3D(this->_v[0]->position, this->_v[1]->position) + BoundingBox3D(this->_v[0]->position, this->_v[2]->position);
	result[1] = result[1] + Point3D(Epsilon, Epsilon, Epsilon);
	this->_bBox = result;
}

void Triangle::initOpenGL( void )
{
	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

double Triangle::intersect( Ray3D ray , RayShapeIntersectionInfo& iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	if (ray.direction.dot(this->mPlane.normal) == 0)
	{
		return Infinity;
	}

	Point3D planePoint = this->mPlane.normal * this->mPlane.distance;
	double t = (planePoint - ray.position).dot(this->mPlane.normal) / ray.direction.dot(this->mPlane.normal);
	if (t < 0)
	{
		return Infinity;
	}
	else if (range.isInside(Point1D(t)) && validityLambda(t))
	{
		Point3D pointIntersect = ray.position + (ray.direction * t);

		double alpha, beta, gamma;
		Point3D side2 = pointIntersect - this->_v[0]->position;
		double d20 = side2.dot(this->side0);
		double d21 = side2.dot(this->side1);
		beta = (this->d11 * d20 - this->d01 * d21) / this->denominator;
		gamma = (this->d00 * d21 - this->d01 * d20) / this->denominator;
		alpha = 1 - beta - gamma;

		if (alpha >= 0 && beta >= 0 && gamma >= 0)
		{
			iInfo.position = pointIntersect;
			iInfo.normal = (this->_v[0]->normal * alpha + this->_v[1]->normal * beta + this->_v[2]->normal * gamma).normalize();
			// iInfo.normal = Point3D::CrossProduct(this->_v[0]->position,this->_v[1]->position).normalize();
			iInfo.texture = alpha * this->_v[0]->texCoordinate + beta * this->_v[1]->texCoordinate + gamma * this->_v[2]->texCoordinate;
			return t;
		}
		else
		{
			return Infinity;
		}
	}
	else
	{
		return Infinity;
	}
}

void Triangle::drawOpenGL( GLSLProgram * glslProgram ) const
{
	const Vertex& v0 = *(this->_v[0]);
	const Vertex& v1 = *(this->_v[1]);
	const Vertex& v2 = *(this->_v[2]);

	glBegin(GL_TRIANGLES);

	glNormal3d(v0.normal[0], v0.normal[1], v0.normal[2]);
	glTexCoord2d(v0.texCoordinate[0], v0.texCoordinate[1]);
	glVertex3d(v0.position[0], v0.position[1], v0.position[2]);

	glNormal3d(v1.normal[0], v1.normal[1], v1.normal[2]);
	glTexCoord2d(v1.texCoordinate[0], v1.texCoordinate[1]);
	glVertex3d(v1.position[0], v1.position[1], v1.position[2]);

	glNormal3d(v2.normal[0], v2.normal[1], v2.normal[2]);
	glTexCoord2d(v2.texCoordinate[0], v2.texCoordinate[1]);
	glVertex3d(v2.position[0], v2.position[1], v2.position[2]);

	glEnd();

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}
