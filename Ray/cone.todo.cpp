#include <cmath>
#include <Util/exceptions.h>
#include <Util/polynomial.h>
#include "scene.h"
#include "cone.h"
#include <vector>
#include <algorithm>

using namespace Ray;
using namespace Util;
using std::vector;

//////////
// Cone //
//////////

void Cone::init( const LocalSceneData &data )
{
	// Set the material pointer
	if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else if( _materialIndex>=data.materials.size() ) THROW( "material index out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else _material = &data.materials[ _materialIndex ];

	//////////////////////////////////
	// Do any necessary set-up here //
	//////////////////////////////////
	this->initializeMesh(Shape::OpenGLTessellationComplexity);
}

void Cone::updateBoundingBox( void )
{
	Point3D p1 = Point3D(this->center[0] - radius, this->center[1], this->center[2] - radius);
	Point3D p2 = Point3D(this->center[0] + radius, this->center[1] + this->height, this->center[2] + radius);
	this->_bBox = BoundingBox3D(p1, p2);
}

void Cone::initOpenGL( void )
{
	/////////////////////////////////////////
	// Do any necessary OpenGL set-up here //
	/////////////////////////////////////////

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

double Cone::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	Ray3D tempRay = ray - this->center;
	double coeff = this->radius * this->radius / this->height / this->height;
	double a = tempRay.direction[0] * tempRay.direction[0] + tempRay.direction[2] * tempRay.direction[2] - coeff * tempRay.direction[1] * tempRay.direction[1];
	double b = 2 * (tempRay.position[0] * tempRay.direction[0] + tempRay.position[2] * tempRay.direction[2] - coeff * tempRay.position[1] * tempRay.direction[1]
		+ coeff * this->height * tempRay.direction[1]);
	double c = tempRay.position[0] * tempRay.position[0] + tempRay.position[2] * tempRay.position[2] - coeff * tempRay.position[1] * tempRay.position[1]
		- this->radius * this->radius + 2 * coeff * this->height * tempRay.position[1];

	double delta = b * b - 4 * a * c;
	if (delta < 0)
	{
		return Infinity;
	}
	else
	{
		double tSide1 = (-b - sqrt(delta)) / (2 * a);
		double tSide2 = (-b + sqrt(delta)) / (2 * a);
		double tBase = (this->center[1] - ray.position[1]) / ray.direction[1];

		vector<double> tempVector = vector<double>();
		tempVector.push_back(tSide1);
		tempVector.push_back(tSide2);
		tempVector.push_back(tBase);
		std::sort(tempVector.begin(), tempVector.end());

		for (int i = 0; i <= 2; i++)
		{
			double t = tempVector[i];
			Point3D intersection = ray.position + (ray.direction * t);
			if (range.isInside(Point1D(t)) && validityLambda(t) 
				&& intersection[1] > this->center[1] - 5 * Epsilon && intersection[1] < this->center[1] + this->height)
			{
				if (t == tSide1 || t == tSide2)
				{
					iInfo.position = intersection;
					iInfo.material = this->_material;
					Point3D n = ((intersection - this->center) * Point3D(1.0, 0.0, 1.0)).normalize();
					n = (n + Point3D(0.0, this->radius / this->height, 0.0)).normalize();
					iInfo.normal = n;
					return t;
				}
				else if (t == tBase)
				{
					if ((intersection - this->center).magnitude() <= this->radius)
					{
						iInfo.position = intersection;
						iInfo.material = this->_material;
						iInfo.normal = Point3D(0.0, -1.0, 0.0);
						return t;
					}
				}
				else
					THROW("ray cylinder intersection failure");
			}
		}

		return Infinity;
	}
}

bool Cone::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	Point3D p1 = p - this->center;
	return (p1[1] > 0 && p1[1] < this->height && p1[0] * p1[0] + p1[2] * p1[2] < pow(this->radius * (this->height - p1[1]) / this->height, 2));
}

void Cone::drawOpenGL( GLSLProgram * glslProgram ) const
{
	this->_material->drawOpenGL(glslProgram);

	glBegin(GL_TRIANGLES);
	for (MeshTriangle const& mt : this->mMesh.mSurfaces)
	{
		glNormal3d(mt.mv0.normal[0], mt.mv0.normal[1], mt.mv0.normal[2]);
		glVertex3d(mt.mv0.position[0], mt.mv0.position[1], mt.mv0.position[2]);

		glNormal3d(mt.mv1.normal[0], mt.mv1.normal[1], mt.mv1.normal[2]);
		glVertex3d(mt.mv1.position[0], mt.mv1.position[1], mt.mv1.position[2]);

		glNormal3d(mt.mv2.normal[0], mt.mv2.normal[1], mt.mv2.normal[2]);
		glVertex3d(mt.mv2.position[0], mt.mv2.position[1], mt.mv2.position[2]);
	}
	glEnd();

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

void Cone::initializeMesh(int complexity)
{
	Mesh& mesh = this->mMesh;
	mesh.mComplexity = complexity;

	Vertex vTop, vBottom;
	vTop.position = this->center + Point3D(0.0, this->height, 0.0);
	vBottom.position = this->center;
	vBottom.normal = Point3D(0.0, -1.0, 0.0);
	
	for (int i = 0; i < complexity * 2; i++)
	{
		double theta1 = Pi / complexity * i;
		double theta2 = Pi / complexity * (i + 1);

		Vertex v1, v2;
		v1.position = this->center + Point3D(this->radius * sin(theta1), 0.0, this->radius * cos(theta1));
		v2.position = this->center + Point3D(this->radius * sin(theta2), 0.0, this->radius * cos(theta2));

		v1.normal = Point3D(0.0, -1.0, 0.0);
		v2.normal = Point3D(0.0, -1.0, 0.0);
		mesh.mSurfaces.push_back(MeshTriangle(vBottom, v2, v1));

		Point3D normal1 = (v1.position - this->center).normalize();
		normal1 = (normal1 + Point3D(0.0, this->radius / this->height, 0.0)).normalize();
		Point3D normal2 = (v2.position - this->center).normalize();
		normal1 = (normal2 + Point3D(0.0, this->radius / this->height, 0.0)).normalize();
		v1.normal = normal1;
		v2.normal = normal2;
		vTop.normal = (normal1 + normal2).normalize();
		mesh.mSurfaces.push_back(MeshTriangle(vTop, v1, v2));
	}
}