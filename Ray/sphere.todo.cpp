#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "sphere.h"

using namespace Ray;
using namespace Util;

////////////
// Sphere //
////////////

void Sphere::init( const LocalSceneData &data )
{
	// Set the material pointer
	if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else if( _materialIndex>=data.materials.size() ) THROW( "material index out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else _material = &data.materials[ _materialIndex ];

	// Initialize triangle mesh
	this->initializeMesh(Shape::OpenGLTessellationComplexity);
}

void Sphere::updateBoundingBox( void )
{
	this->_bBox = BoundingBox3D(this->center - Point3D(this->radius, this->radius, this->radius), 
		this->center + Point3D(this->radius, this->radius, this->radius));
}

void Sphere::initOpenGL( void )
{
	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

double Sphere::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	double a = 1;
	double b = ray.direction.dot(ray.position - this->center) * 2;
	double c = (ray.position - this->center).dot(ray.position - this->center) - this->radius * this->radius;

	double delta = b * b - 4 * a * c;
	if (delta < 0)
	{
		return Infinity;
	}
	else
	{
		double t1 = (-b - sqrt(delta)) / (2 * a);
		double t2 = (-b + sqrt(delta)) / (2 * a);
		double smaller = min(t1, t2);
		double greater = max(t1, t2);
		if (range.isInside(Point1D(smaller)) && validityLambda(smaller))
		{
			iInfo.material = this->_material;
			Point3D p = ray.position + (ray.direction * smaller);
			iInfo.position = p;
			iInfo.normal = (p - this->center).normalize();
			return smaller;
		}
		else if (range.isInside(Point1D(greater)) && validityLambda(greater))
		{
			iInfo.material = this->_material;
			Point3D p = ray.position + (ray.direction * greater);
			iInfo.position = p;
			iInfo.normal = (p - this->center).normalize();
			return greater;
		}
		else
		{
			return Infinity;
		}
	}
}

bool Sphere::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	Point3D p1 = p - this->center;
	return (p1.squareNorm() < this->radius * this->radius);
}

void Sphere::drawOpenGL( GLSLProgram * glslProgram ) const
{
	this->_material->drawOpenGL(glslProgram);

	for (MeshTriangle const& mt : this->mMesh.mSurfaces)
	{
		glBegin(GL_TRIANGLES);

		glNormal3d(mt.mv0.normal[0], mt.mv0.normal[1], mt.mv0.normal[2]);
		glVertex3d(mt.mv0.position[0], mt.mv0.position[1], mt.mv0.position[2]);

		glNormal3d(mt.mv1.normal[0], mt.mv1.normal[1], mt.mv1.normal[2]);
		glVertex3d(mt.mv1.position[0], mt.mv1.position[1], mt.mv1.position[2]);

		glNormal3d(mt.mv2.normal[0], mt.mv2.normal[1], mt.mv2.normal[2]);
		glVertex3d(mt.mv2.position[0], mt.mv2.position[1], mt.mv2.position[2]);

		glEnd();
	}

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

void Sphere::initializeMesh(int complexity)
{
	Mesh& mesh = this->mMesh;
	mesh.mComplexity = complexity;

	for (int i = 0; i < complexity; i++)
	{
		for (int j = 0; j < 2 * complexity; j++)
		{
			double phi1 = Pi / complexity * i;
			double phi2 = Pi / complexity * (i + 1);
			double theta1 = Pi / complexity * j;
			double theta2 = Pi / complexity * (j + 1);

			Vertex v1, v2, v3, v4;
			double r = this->radius;

			v1.position = this->center + Point3D(sin(phi1) * sin(theta1), cos(phi1), sin(phi1) * cos(theta1)) * r;
			v2.position = this->center + Point3D(sin(phi1) * sin(theta2), cos(phi1), sin(phi1) * cos(theta2)) * r;
			v3.position = this->center + Point3D(sin(phi2) * sin(theta1), cos(phi2), sin(phi2) * cos(theta1)) * r;
			v4.position = this->center + Point3D(sin(phi2) * sin(theta2), cos(phi2), sin(phi2) * cos(theta2)) * r;
			v1.normal = (v1.position - this->center).normalize();
			v2.normal = (v2.position - this->center).normalize();
			v3.normal = (v3.position - this->center).normalize();
			v4.normal = (v4.position - this->center).normalize();

			mesh.mSurfaces.push_back(MeshTriangle(v1, v3, v2));
			mesh.mSurfaces.push_back(MeshTriangle(v2, v3, v4));
		}
	}
}