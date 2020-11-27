#ifndef BOX_INCLUDED
#define BOX_INCLUDED
#include <Util/geometry.h>
#include "shape.h"
#include "scene.h"

namespace Ray
{
	/** This class represents a box and is defined by its center and the length of the sides. */
	class Box : public Shape
	{
		/** The index of the material associated with the box */
		int _materialIndex;

		/** The material associated with the box */
		const class Material *_material;
	public:
		/** The center of the box */
		Util::Point3D center;

		/** The half lengths of the sides of the box */
		Util::Point3D length;

		/** This static method returns the directive describing the shape. */
		static std::string Directive( void ){ return "shape_box"; }

		/** The default constructor */
		Box( void );

		/** The triangle mesh associated with the sphere */
		Mesh mMesh;

		///////////////////
		// Shape methods //
		///////////////////
	private:
		void _write( std::ostream &stream ) const;
		void _read( std::istream &stream );
	public:
		std::string name( void ) const { return "box"; }
		void init( const class LocalSceneData& data  );
		void initOpenGL( void );
		void updateBoundingBox( void );
		double intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , Util::BoundingBox1D range = Util::BoundingBox1D( Util::Epsilon , Util::Infinity ) , std::function< bool (double) > validityFunction = [] ( double t ){ return true; } ) const;
		bool isInside( Util::Point3D p ) const;
		void drawOpenGL( GLSLProgram *glslProgram ) const;
		void initializeMesh(void);
	};
}
#endif // BOX_INCLUDED

