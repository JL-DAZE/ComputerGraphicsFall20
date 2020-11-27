#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "cylinder.h"
#include <vector>
#include <algorithm>

using namespace Ray;
using namespace Util;
using std::vector;

//////////////
// Cylinder //
//////////////

void Cylinder::init( const LocalSceneData &data )
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

void Cylinder::updateBoundingBox( void )
{
	Point3D p1 = Point3D(this->center[0] - this->radius, this->center[1], this->center[2] - this->radius);
	Point3D p2 = Point3D(this->center[0] + this->radius, this->center[1] + this->height, this->center[2] + this->radius);
	this->_bBox = BoundingBox3D(p1, p2);
}

void Cylinder::initOpenGL( void )
{
	/////////////////////////////////////////
	// Do any necessary OpenGL set-up here //
	/////////////////////////////////////////

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

double Cylinder::intersect( Ray3D ray , RayShapeIntersectionInfo& iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	Ray3D tempRay = ray - this->center;
	double a = tempRay.direction[0] * tempRay.direction[0] + tempRay.direction[2] * tempRay.direction[2];
	double b = 2 * (tempRay.position[0] * tempRay.direction[0] + tempRay.position[2] * tempRay.direction[2]);
	double c = tempRay.position[0] * tempRay.position[0] + tempRay.position[2] * tempRay.position[2] - this->radius * this->radius;

	double delta = b * b - 4 * a * c;
	if (delta < 0)
	{
		return Infinity;
	}
	else
	{
		double tSide1 = (-b - sqrt(delta)) / (2 * a);
		double tSide2 = (-b + sqrt(delta)) / (2 * a);
		double tCapUp = (this->center[1] + this->height - ray.position[1]) / ray.direction[1];
		double tCapBottom = (this->center[1] - ray.position[1]) / ray.direction[1];

		BoundingBox1D tempBBox = BoundingBox1D(Point1D(tSide1), Point1D(tSide2)) ^ BoundingBox1D(Point1D(tCapUp), Point1D(tCapBottom));
		if (tempBBox.isEmpty())
			return Infinity;

		vector<double> tempVector = vector<double>();
		tempVector.push_back(tSide1);
		tempVector.push_back(tSide2);
		tempVector.push_back(tCapUp);
		tempVector.push_back(tCapBottom);
		std::sort(tempVector.begin(), tempVector.end());

		for (int i = 0; i <= 3; i++)
		{
			double t = tempVector[i];
			if (range.isInside(Point1D(t)) && validityLambda(t) && tempBBox.isInsideOrOnBoundary(Point1D(t)))
			{
				Point3D intersection = ray.position + (ray.direction * t);
				iInfo.position = intersection;
				iInfo.material = this->_material;
				if (t == tSide1 || t == tSide2)
					iInfo.normal = (intersection * Point3D(1.0, 0.0, 1.0) - this->center).normalize();
				else if (t == tCapUp)
					iInfo.normal = Point3D(0.0, 1.0, 0.0);
				else if (t == tCapBottom)
					iInfo.normal = Point3D(0.0, -1.0, 0.0);
				else
					THROW("ray cylinder intersection failure");

				return t;
			}
		}
		
		return Infinity;
	}
}

bool Cylinder::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	Point3D p1 = p - this->center;
	return (p1[1] > -this->height / 2 && p1[1] < this->height / 2 && p1[0] * p1[0] + p1[2] * p1[2] < this->radius * this->radius);
}

void Cylinder::drawOpenGL( GLSLProgram *glslProgram ) const
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
}

void Cylinder::initializeMesh(int complexity)
{
	Mesh& mesh = this->mMesh;
	mesh.mComplexity = complexity;

	Vertex vTop, vBottom;
	vBottom.position = this->center;
	vBottom.normal = Point3D(0.0, -1.0, 0.0);
	vTop.position = this->center + Point3D(0.0, this->height, 0.0);
	vTop.normal = Point3D(0.0, 1.0, 0.0);

	for (int i = 0; i < 2 * complexity; i++)
	{
		double theta1 = Pi / complexity * i;
		double theta2 = Pi / complexity * (i + 1);

		Vertex v1, v2, v3, v4;
		v1.position = this->center + Point3D(this->radius * sin(theta1), this->height, this->radius * cos(theta1));
		v1.normal = Point3D(0.0, 1.0, 0.0);
		v2.position = this->center + Point3D(this->radius * sin(theta2), this->height, this->radius * cos(theta2));
		v2.normal = Point3D(0.0, 1.0, 0.0);
		v3.position = this->center + Point3D(this->radius * sin(theta1), 0.0, this->radius * cos(theta1));
		v3.normal = Point3D(0.0, -1.0, 0.0);
		v4.position = this->center + Point3D(this->radius * sin(theta2), 0.0, this->radius * cos(theta2));
		v4.normal = Point3D(0.0, -1.0, 0.0);

		mesh.mSurfaces.push_back(MeshTriangle(vTop, v1, v2));
		mesh.mSurfaces.push_back(MeshTriangle(vBottom, v4, v3));

		v1.normal = (v3.position - this->center).normalize();
		v3.normal = (v3.position - this->center).normalize();
		v2.normal = (v4.position - this->center).normalize();
		v4.normal = (v4.position - this->center).normalize();

		mesh.mSurfaces.push_back(MeshTriangle(v1, v3, v2));
		mesh.mSurfaces.push_back(MeshTriangle(v2, v3, v4));
	}
}
