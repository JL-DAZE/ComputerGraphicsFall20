#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "torus.h"
#include <vector>
#include <algorithm>

using namespace Ray;
using namespace Util;
using std::vector;

///////////
// Torus //
///////////

void Torus::init( const LocalSceneData &data )
{
	// Set the material pointer
	if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else if( _materialIndex>=data.materials.size() ) THROW( "material index out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else _material = &data.materials[ _materialIndex ];

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
	this->cRadius = (this->iRadius + this->oRadius) / 2;
	this->rRadius = (this->iRadius - this->oRadius) / 2;

	this->poly = Polynomial3D<4>();
	double RrSquareDiff = this->cRadius * this->cRadius - this->rRadius * this->rRadius;
	this->poly.coefficient(4u, 0u, 0u) = 1;
	this->poly.coefficient(0u, 4u, 0u) = 1;
	this->poly.coefficient(0u, 0u, 4u) = 1;
	this->poly.coefficient(2u, 0u, 0u) = 2 * RrSquareDiff - 4 * this->cRadius * this->cRadius;
	this->poly.coefficient(0u, 2u, 0u) = 2 * RrSquareDiff;
	this->poly.coefficient(0u, 0u, 2u) = 2 * RrSquareDiff - 4 * this->cRadius * this->cRadius;
	this->poly.coefficient(2u, 2u, 0u) = 2;
	this->poly.coefficient(2u, 0u, 2u) = 2;
	this->poly.coefficient(0u, 2u, 2u) = 2;
	this->poly.coefficient(0u, 0u, 0u) = RrSquareDiff * RrSquareDiff;

	this->initializeMesh(Shape::OpenGLTessellationComplexity);
}

void Torus::updateBoundingBox( void )
{
	Point3D p1 = this->center - Point3D(this->cRadius, this->rRadius, this->cRadius);
	Point3D p2 = this->center + Point3D(this->cRadius, this->rRadius, this->cRadius);
	this->_bBox = BoundingBox3D(p1, p2);
}

void Torus::initOpenGL( void )
{
	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

double Torus::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	Ray3D tempRay = ray - this->center;
	Polynomial1D<4> evalPoly = this->poly(tempRay);
	double rootArray[4];
	int numRoots = evalPoly.roots(rootArray);

	vector<double> tempVector = vector<double>();
	for (int i = 0; i < numRoots; i++)
		tempVector.push_back(rootArray[i]);
	std::sort(tempVector.begin(), tempVector.end());

	for (vector<double>::const_iterator it = tempVector.cbegin(); it != tempVector.cend(); ++it)
	{
		double t = *it;
		if (range.isInside(t) && validityLambda(t))
		{
			Point3D intersection = ray.position + (ray.direction * t);
			iInfo.position = intersection;
			iInfo.material = this->_material;

			Point3D tempP = intersection - this->center;
			Point3D n = (tempP * Point3D(1.0, 0.0, 1.0)).normalize();
			if (tempP[0] * tempP[0] + tempP[2] * tempP[2] == this->cRadius * this->cRadius)
			{
				iInfo.normal = (tempP[1] > 0) ? Point3D(0.0, 1.0, 0.0) : Point3D(0.0, -1.0, 0.0);
				return t;
			}
			else if (tempP[0] * tempP[0] + tempP[2] * tempP[2] < this->cRadius * this->cRadius)
				n = n * -1;

			double tempValue = sqrt(this->rRadius * this->rRadius - tempP[1] * tempP[1]);
			n = (n + Point3D(0.0, tempP[1] / tempValue, 0.0)).normalize();
			iInfo.normal = n;
			return t;
		}
	}

	return Infinity;
}

bool Torus::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	Point3D p1 = p - this->center;
	return (pow(sqrt(p1[0] * p1[0] + p1[2] * p1[2]) - this->cRadius * this->cRadius, 2) + p1[1] * p1[1] < this->rRadius * this->rRadius);
}

void Torus::drawOpenGL( GLSLProgram * glslProgram ) const
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

void Torus::initializeMesh(int complexity)
{
	Mesh& mesh = this->mMesh;
	mesh.mComplexity = complexity;
	double majorR = this->cRadius;
	double minorR = this->rRadius;

	for (int i = 0; i < 2 * complexity; i++)
	{
		for (int j = 0; j < 2 * complexity; j++)
		{
			double phi1 = Pi / complexity * i;
			double phi2 = Pi / complexity * (i + 1);
			double theta1 = Pi / complexity * j;
			double theta2 = Pi / complexity * (j + 1);

			Vertex v1, v2, v3, v4;
			
			v1.position = this->center + Point3D((majorR + minorR * sin(theta1)) * sin(phi1), minorR * cos(theta1), (majorR + minorR * sin(theta1)) * cos(phi1));
			v2.position = this->center + Point3D((majorR + minorR * sin(theta1)) * sin(phi2), minorR * cos(theta1), (majorR + minorR * sin(theta1)) * cos(phi2));
			v3.position = this->center + Point3D((majorR + minorR * sin(theta2)) * sin(phi1), minorR * cos(theta2), (majorR + minorR * sin(theta2)) * cos(phi1));
			v4.position = this->center + Point3D((majorR + minorR * sin(theta2)) * sin(phi2), minorR * cos(theta2), (majorR + minorR * sin(theta2)) * cos(phi2));
			v1.normal = Point3D(sin(theta1) * sin(phi1), cos(theta1), sin(theta1) * cos(phi1));
			v2.normal = Point3D(sin(theta1) * sin(phi2), cos(theta1), sin(theta1) * cos(phi2));
			v3.normal = Point3D(sin(theta2) * sin(phi1), cos(theta2), sin(theta2) * cos(phi1));
			v4.normal = Point3D(sin(theta2) * sin(phi2), cos(theta2), sin(theta2) * cos(phi2));

			mesh.mSurfaces.push_back(MeshTriangle(v1, v3, v2));
			mesh.mSurfaces.push_back(MeshTriangle(v2, v3, v4));
		}
	}
}