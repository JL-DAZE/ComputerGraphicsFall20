#include <Util/exceptions.h>
#include "shapeList.h"
#include "triangle.h"
#include <vector>
#include <algorithm>

using namespace Ray;
using namespace Util;
using std::vector;

////////////////
// Difference //
////////////////
void Difference::updateBoundingBox( void )
{
	this->_shape0->updateBoundingBox();
	this->_shape1->updateBoundingBox();
	this->_bBox = this->_shape0->boundingBox() + this->_shape1->boundingBox();
}

double Difference::intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	Ray3D tempRay = ray;
	RayShapeIntersectionInfo tempIInfo0 = RayShapeIntersectionInfo();
	RayShapeIntersectionInfo tempIInfo1 = RayShapeIntersectionInfo();
	Point3D epsilonV = ray.direction * Epsilon;
	while (1)
	{
		double t0 = this->_shape0->intersect(tempRay, tempIInfo0, range, validityLambda);
		double t1 = this->_shape1->intersect(tempRay, tempIInfo1, range, validityLambda);
		if (t0 == Infinity)
			break;

		double smaller = std::min(t0, t1);
		RayShapeIntersectionInfo smallerIInfo = (smaller == t0) ? tempIInfo0 : tempIInfo1;
		Point3D tempP = tempRay.position + (tempRay.direction * smaller);
		if (this->isInside(tempP + epsilonV) || this->isInside(tempP - epsilonV))
		{
			double result = (smallerIInfo.position[0] - ray.position[0]) / ray.direction[0];
			if (range.isInside(Point1D(result)) && validityLambda(result))
			{
				iInfo = smallerIInfo;
				if (smaller == t1)
					iInfo.normal = iInfo.normal * -1;
				return result;
			}
		}
		else
			tempRay = Ray3D(tempP + epsilonV, ray.direction);
	}
	return Infinity;
}

bool Difference::isInside( Util::Point3D p ) const
{
	return (this->_shape0->isInside(p) && !this->_shape1->isInside(p));
}

///////////////
// ShapeList //
///////////////
double ShapeList::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	double result = Infinity;
	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	vector<ShapeBoundingBoxHit> hitList = vector<ShapeBoundingBoxHit>();

	for (vector<Shape*>::const_iterator it = this->shapes.cbegin(); it != this->shapes.cend(); ++it)
	{
		BoundingBox1D bBox = (**it).boundingBox().intersect(ray);
		if (!bBox.isEmpty())
		{
			ShapeBoundingBoxHit hit = ShapeBoundingBoxHit();
			hit.t = bBox[0][0];
			hit.shape = *it;
			hitList.push_back(hit);
		}
	}

	std::sort(hitList.begin(), hitList.end(), ShapeBoundingBoxHit::Compare);
	for (vector<ShapeBoundingBoxHit>::const_iterator it = hitList.cbegin(); it != hitList.cend(); ++it)
	{
		if (it->t <= result)
		{
			RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
			double tempT = it->shape->intersect(ray, tempIInfo, range, validityLambda);
			if (tempT < result)
			{
				result = tempT;
				iInfo = tempIInfo;
			}
		}
		else
			break;
	}

	return result;
}

bool ShapeList::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	for (vector<Shape*>::const_iterator it = this->shapes.cbegin(); it != this->shapes.cend(); ++it)
		if ((**it).isInside(p))
			return true;
	return false;
}

void ShapeList::init( const LocalSceneData &data )
{
	// Initialize the children
	for( int i=0 ; i<shapes.size() ; i++ ) shapes[i]->init( data );

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
}

void ShapeList::updateBoundingBox( void )
{
	BoundingBox3D result;
	for (vector<Shape*>::iterator it = this->shapes.begin(); it != this->shapes.end(); ++it)
	{
		(**it).updateBoundingBox();
		if (it == this->shapes.begin())
			result = (**it).boundingBox();
		else
			result += (**it).boundingBox();
	}
	this->_bBox = result;
}

void ShapeList::initOpenGL( void )
{
	// Initialize the children
	for( int i=0 ; i<shapes.size() ; i++ ) shapes[i]->initOpenGL();

	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////
}

void ShapeList::drawOpenGL( GLSLProgram * glslProgram ) const
{
	for (vector<Shape*>::const_iterator it = this->shapes.cbegin(); it != this->shapes.cend(); ++it)
	{
		(**it).drawOpenGL(glslProgram);
	}

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

/////////////////
// AffineShape //
/////////////////
double AffineShape::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	Matrix3D matrixInverseLinear = Matrix3D(this->getInverseMatrix());
	Point3D transformedRayPosition = this->getInverseMatrix() * ray.position;
	Point3D transformedRayDirection = (matrixInverseLinear * ray.direction).normalize();
	Ray3D transformedRay = Ray3D(transformedRayPosition, transformedRayDirection);

	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	double t = _shape->intersect(transformedRay, tempIInfo, range, validityLambda);
	
	if (t < Infinity)
	{
		iInfo.material = tempIInfo.material;
		iInfo.position = this->getMatrix() * tempIInfo.position;
		iInfo.normal = (this->getNormalMatrix() * tempIInfo.normal).normalize();
		iInfo.texture = tempIInfo.texture;
		return (iInfo.position - ray.position).magnitude();
	}
	else
		return Infinity;
}

bool AffineShape::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	Point3D p1 = this->getInverseMatrix() * p;
	return this->_shape->isInside(p1);
}

void AffineShape::updateBoundingBox( void )
{
	this->_shape->updateBoundingBox();
	this->_bBox = this->getMatrix() * this->_shape->boundingBox();
}

void AffineShape::drawOpenGL( GLSLProgram * glslProgram ) const
{
	Matrix4D tempM = this->getMatrix();
	double matrix[16] = { tempM(0, 0), tempM(1, 0), tempM(2, 0), tempM(3, 0),
						  tempM(0, 1), tempM(1, 1), tempM(2, 1), tempM(3, 1),
						  tempM(0, 2), tempM(1, 2), tempM(2, 2), tempM(3, 2),
						  tempM(0, 3), tempM(1, 3), tempM(2, 3), tempM(3, 3) };

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	// glLoadMatrixd(matrix);
	// glLoadIdentity();
	glMultMatrixd(matrix);
	this->_shape->drawOpenGL(glslProgram);
	glPopMatrix();

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();
}

//////////////////
// TriangleList //
//////////////////
double TriangleList::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	double result = Infinity;
	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();

	for (vector<Shape*>::const_iterator it = this->_shapeList.shapes.cbegin(); it != this->_shapeList.shapes.cend(); ++it)
	{
		double t = (**it).intersect(ray, tempIInfo, range, validityLambda);
		if (t < result)
		{
			result = t;
			iInfo = tempIInfo;
			iInfo.material = this->_material;
		}
	}

	return result;
}

void TriangleList::drawOpenGL( GLSLProgram * glslProgram ) const
{
	this->_material->drawOpenGL(glslProgram);
	for (vector<Shape*>::const_iterator it = this->_shapeList.shapes.cbegin(); it != this->_shapeList.shapes.cend(); ++it)
	{
		(**it).drawOpenGL(glslProgram);
	}
		
	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

void TriangleList::init( const LocalSceneData &data )
{
	// Set the vertex and material pointers
	_vertices = &data.vertices[0];
	_vNum = (unsigned int)data.vertices.size();
	if( _materialIndex>=data.materials.size() ) THROW( "shape specifies a material that is out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else _material = &data.materials[ _materialIndex ];

	_shapeList.init( data );
}

void TriangleList::initOpenGL( void )
{
	_shapeList.initOpenGL();

	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

///////////
// Union //
///////////
double Union::intersect(Ray3D ray, RayShapeIntersectionInfo& iInfo, BoundingBox1D range, std::function< bool(double) > validityLambda) const
{
	double result = Infinity;
	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	vector<ShapeBoundingBoxHit> hitList = vector<ShapeBoundingBoxHit>();

	for (vector<Shape*>::const_iterator it = this->_shapeList.shapes.cbegin(); it != this->_shapeList.shapes.cend(); ++it)
	{
		BoundingBox1D bBox = (**it).boundingBox().intersect(ray);
		if (!bBox.isEmpty())
		{
			ShapeBoundingBoxHit hit = ShapeBoundingBoxHit();
			hit.t = bBox[0][0];
			hit.shape = *it;
			hitList.push_back(hit);
		}
	}

	std::sort(hitList.begin(), hitList.end(), ShapeBoundingBoxHit::Compare);
	for (vector<ShapeBoundingBoxHit>::const_iterator it = hitList.cbegin(); it != hitList.cend(); ++it)
	{
		if (it->t <= result)
		{
			RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
			double tempT = it->shape->intersect(ray, tempIInfo, range, validityLambda);
			Point3D epsilonVector = ray.direction * Epsilon;
			if (tempT < result && (!this->isInside(tempIInfo.position + epsilonVector) || !this->isInside(tempIInfo.position - epsilonVector)))
			{
				result = tempT;
				iInfo = tempIInfo;
			}
		}
		else
			break;
	}

	return result;
}

void Union::init( const LocalSceneData &data )
{
	_shapeList.init( data );

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
}

void Union::updateBoundingBox( void )
{
	this->_shapeList.updateBoundingBox();
	this->_bBox = this->_shapeList.boundingBox();
}

bool Union::isInside( Point3D p ) const
{
	return this->_shapeList.isInside(p);
}

//////////////////
// Intersection //
//////////////////
double Intersection::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	double result = Infinity;
	RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
	vector<ShapeBoundingBoxHit> hitList = vector<ShapeBoundingBoxHit>();

	for (vector<Shape*>::const_iterator it = this->_shapeList.shapes.cbegin(); it != this->_shapeList.shapes.cend(); ++it)
	{
		BoundingBox1D bBox = (**it).boundingBox().intersect(ray);
		if (!bBox.isEmpty())
		{
			ShapeBoundingBoxHit hit = ShapeBoundingBoxHit();
			hit.t = bBox[0][0];
			hit.shape = *it;
			hitList.push_back(hit);
		}
	}

	std::sort(hitList.begin(), hitList.end(), ShapeBoundingBoxHit::Compare);
	for (vector<ShapeBoundingBoxHit>::const_iterator it = hitList.cbegin(); it != hitList.cend(); ++it)
	{
		if (it->t <= result)
		{
			RayShapeIntersectionInfo tempIInfo = RayShapeIntersectionInfo();
			double tempT = it->shape->intersect(ray, tempIInfo, range, validityLambda);
			Point3D epsilonVector = ray.direction * Epsilon;
			if (tempT < result && (this->isInside(tempIInfo.position + epsilonVector) || this->isInside(tempIInfo.position - epsilonVector)))
			{
				result = tempT;
				iInfo = tempIInfo;
			}
		}
		else
			break;
	}

	return result;
}

void Intersection::init( const LocalSceneData &data )
{
	_shapeList.init( data );

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
}

void Intersection::updateBoundingBox(void)
{
	this->_shapeList.updateBoundingBox();
	BoundingBox3D bBox;
	for (vector<Shape*>::iterator it = this->_shapeList.shapes.begin(); it != this->_shapeList.shapes.end(); ++it)
	{
		if (it == this->_shapeList.shapes.begin())
			bBox = (**it).boundingBox();
		else
			bBox = bBox ^ (**it).boundingBox();
	}
	this->_bBox = bBox;
}

bool Intersection::isInside( Point3D p ) const
{
	if (!this->boundingBox().isInside(p))
		return false;
	for (vector<Shape*>::const_iterator it = this->_shapeList.shapes.cbegin(); it != this->_shapeList.shapes.cend(); ++it)
		if (!(**it).isInside(p))
			return false;
	return true;
}

