#ifndef CYLINDER_INCLUDED
#define CYLINDER_INCLUDED
#include <Util/geometry.h>
#include <Util/polynomial.h>
#include "shape.h"
#include "scene.h"

namespace Ray
{
	/** This class represents a cylinder whose central axis is parallel to the y-axis, 
	* and is defined by the center of the cylinder, the height from the top cap
	* to the bottom cap, and the radius of the cylinder. */
	class Cylinder : public Shape
	{
		/** The OpenGL vertex buffer identifier */
		GLuint _vertexBufferID = 0;

		/** The OpenGL element buffer identifier */
		GLuint _elementBufferID = 0;

		/** The index of the material associated with the box */
		int _materialIndex;

		/** The material associated with the cylinder */
		const class Material *_material;
	public:
		/** The center of the cylinder */
		Util::Point3D center;

		/** The hieght of the cylinder */
		double height;

		/** The radius of the cylinder */
		double radius;

		/** This static method returns the directive describing the shape. */
		static std::string Directive( void ){ return "shape_cylinder"; }

		/** The default constructor */
		Cylinder( void );

		/** The triangle mesh associated with the sphere */
		Mesh mMesh;

		///////////////////
		// Shape methods //
		///////////////////
	private:
		void _write( std::ostream &stream ) const;
		void _read( std::istream &stream );
	public:
		std::string name( void ) const { return "cylinder"; }
		void init( const class LocalSceneData &data );
		void initOpenGL( void );
		void updateBoundingBox( void );
		double intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , Util::BoundingBox1D range = Util::BoundingBox1D( Util::Epsilon , Util::Infinity ) , std::function< bool (double) > validityFunction = [] ( double t ){ return true; } ) const;
		bool isInside( Util::Point3D p ) const;
		void drawOpenGL( GLSLProgram * glslProgram ) const;
		void initializeMesh(int complexity);
	};
}
#endif // CYLINDER_INCLUDED 
