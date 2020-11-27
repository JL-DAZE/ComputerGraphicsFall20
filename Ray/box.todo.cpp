#include <cmath>
#include  <Util/exceptions.h>
#include "scene.h"
#include "box.h"

using namespace Ray;
using namespace Util;

/////////
// Box //
/////////

void Box::init( const LocalSceneData& data )
{
	// Set the material pointer
	if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else if( _materialIndex>=data.materials.size() ) THROW( "material index out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else _material = &data.materials[ _materialIndex ];

	//////////////////////////////////
	// Do any necessary set-up here //
	//////////////////////////////////
	this->initializeMesh();
}

void Box::updateBoundingBox( void )
{
	this->_bBox = BoundingBox3D(this->center - this->length, this->center + this->length);
}

void Box::initOpenGL( void )
{
	/////////////////////////////////////////
	// Do any necessary OpenGL set-up here //
	/////////////////////////////////////////

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

double Box::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	BoundingBox1D bBox = this->boundingBox().intersect(ray);

	double result = bBox[0][0];
	if (bBox.isEmpty())
		return Infinity;
	if (!(range.isInside(result) && validityLambda(result)))
		return Infinity;

	Point3D intersection = ray.position + (ray.direction * result);
	iInfo.position = intersection;
	iInfo.material = this->_material;
	Point3D nearPoint = this->center - this->length;
	Point3D farPoint = this->center + this->length;
	Point3D diffNear = intersection - nearPoint;
	Point3D diffFar = intersection - farPoint;

	double threshold = 5 * Epsilon;
	if (fabs(diffNear[0]) <= threshold)
		iInfo.normal = Point3D(-1.0, 0.0, 0.0);
	else if (fabs(diffNear[1]) <= threshold)
		iInfo.normal = Point3D(0.0, -1.0, 0.0);
	else if (fabs(diffNear[2]) <= threshold)
		iInfo.normal = Point3D(0.0, 0.0, -1.0);
	else if (fabs(diffFar[0]) <= threshold)
		iInfo.normal = Point3D(1.0, 0.0, 0.0);
	else if (fabs(diffFar[1]) <= threshold)
		iInfo.normal = Point3D(0.0, 1.0, 0.0);
	else if (fabs(diffFar[2]) <= threshold)
		iInfo.normal = Point3D(0.0, 0.0, 1.0);
	else
		THROW("ray box intersection failure");

	return result;
}

bool Box::isInside( Point3D p ) const
{
	return this->boundingBox().isInside(p);
}

void Box::drawOpenGL( GLSLProgram * glslProgram ) const
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

void Box::initializeMesh(void)
{
	Mesh& mesh = this->mMesh;

	Vertex v1, v2, v3, v4;
	v1.position = this->center + (this->length * Point3D(-1.0, 1.0, 1.0));
	v2.position = this->center + (this->length * Point3D(1.0, 1.0, 1.0));
	v3.position = this->center + (this->length * Point3D(-1.0, -1.0, 1.0));
	v4.position = this->center + (this->length * Point3D(1.0, -1.0, 1.0));
	v1.normal = Point3D(0.0, 0.0, 1.0);
	v2.normal = Point3D(0.0, 0.0, 1.0);
	v3.normal = Point3D(0.0, 0.0, 1.0);
	v4.normal = Point3D(0.0, 0.0, 1.0);
	mesh.mSurfaces.push_back(MeshTriangle(v1, v3, v2));
	mesh.mSurfaces.push_back(MeshTriangle(v2, v3, v4));

	Vertex v5, v6, v7, v8;
	v5.position = this->center + (this->length * Point3D(1.0, 1.0, -1.0));
	v6.position = this->center + (this->length * Point3D(-1.0, 1.0, -1.0));
	v7.position = this->center + (this->length * Point3D(1.0, -1.0, -1.0));
	v8.position = this->center + (this->length * Point3D(-1.0, -1.0, -1.0));
	v5.normal = Point3D(0.0, 0.0, -1.0);
	v6.normal = Point3D(0.0, 0.0, -1.0);
	v7.normal = Point3D(0.0, 0.0, -1.0);
	v8.normal = Point3D(0.0, 0.0, -1.0);
	mesh.mSurfaces.push_back(MeshTriangle(v5, v7, v6));
	mesh.mSurfaces.push_back(MeshTriangle(v6, v7, v8));

	Vertex v9, v10, v11, v12;
	v9.position = this->center + (this->length * Point3D(-1.0, 1.0, -1.0));
	v10.position = this->center + (this->length * Point3D(1.0, 1.0, -1.0));
	v11.position = this->center + (this->length * Point3D(-1.0, 1.0, 1.0));
	v12.position = this->center + (this->length * Point3D(1.0, 1.0, 1.0));
	v9.normal = Point3D(0.0, 1.0, 0.0);
	v10.normal = Point3D(0.0, 1.0, 0.0);
	v11.normal = Point3D(0.0, 1.0, 0.0);
	v12.normal = Point3D(0.0, 1.0, 0.0);
	mesh.mSurfaces.push_back(MeshTriangle(v9, v11, v10));
	mesh.mSurfaces.push_back(MeshTriangle(v10, v11, v12));

	Vertex v13, v14, v15, v16;
	v13.position = this->center + (this->length * Point3D(-1.0, -1.0, 1.0));
	v14.position = this->center + (this->length * Point3D(1.0, -1.0, 1.0));
	v15.position = this->center + (this->length * Point3D(-1.0, -1.0, -1.0));
	v16.position = this->center + (this->length * Point3D(1.0, -1.0, -1.0));
	v13.normal = Point3D(0.0, -1.0, 0.0);
	v14.normal = Point3D(0.0, -1.0, 0.0);
	v15.normal = Point3D(0.0, -1.0, 0.0);
	v16.normal = Point3D(0.0, -1.0, 0.0);
	mesh.mSurfaces.push_back(MeshTriangle(v13, v15, v14));
	mesh.mSurfaces.push_back(MeshTriangle(v14, v15, v16));

	Vertex v17, v18, v19, v20;
	v17.position = this->center + (this->length * Point3D(1.0, 1.0, 1.0));
	v18.position = this->center + (this->length * Point3D(1.0, 1.0, -1.0));
	v19.position = this->center + (this->length * Point3D(1.0, -1.0, 1.0));
	v20.position = this->center + (this->length * Point3D(1.0, -1.0, -1.0));
	v17.normal = Point3D(1.0, 0.0, 0.0);
	v18.normal = Point3D(1.0, 0.0, 0.0);
	v19.normal = Point3D(1.0, 0.0, 0.0);
	v20.normal = Point3D(1.0, 0.0, 0.0);
	mesh.mSurfaces.push_back(MeshTriangle(v17, v19, v18));
	mesh.mSurfaces.push_back(MeshTriangle(v18, v19, v20));

	Vertex v21, v22, v23, v24;
	v21.position = this->center + (this->length * Point3D(-1.0, 1.0, -1.0));
	v22.position = this->center + (this->length * Point3D(-1.0, 1.0, 1.0));
	v23.position = this->center + (this->length * Point3D(-1.0, -1.0, -1.0));
	v24.position = this->center + (this->length * Point3D(-1.0, -1.0, 1.0));
	v21.normal = Point3D(-1.0, 0.0, 0.0);
	v22.normal = Point3D(-1.0, 0.0, 0.0);
	v23.normal = Point3D(-1.0, 0.0, 0.0);
	v24.normal = Point3D(-1.0, 0.0, 0.0);
	mesh.mSurfaces.push_back(MeshTriangle(v21, v23, v22));
	mesh.mSurfaces.push_back(MeshTriangle(v22, v23, v24));
}