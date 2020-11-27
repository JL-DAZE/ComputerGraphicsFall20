#ifndef RAY_FILE_INSTANCE_INCLUDED
#define RAY_FILE_INSTANCE_INCLUDED
#include <Util/geometry.h>
#include "shape.h"
#include "scene.h"

namespace Ray
{
	/** This subclass of RayShape stores a reference to a .ray file included in the scene-graph.*/
	class FileInstance : public Shape
	{
		/** The index of the file associated to the instance */
		int _fileIndex;

		/** The file associated to the instance */
		const File *_file;
	public:
		/** This static method returns the directive describing the shape. */
		static std::string Directive( void ){ return "ray_file_instance"; }

		/** The default constructor */
		FileInstance( void );

		///////////////////
		// Shape methods //
		///////////////////
	private:
		void _write( std::ostream &stream ) const;
		void _read( std::istream &stream );
	public:
		std::string name( void ) const { return "file instance"; }
		void init( const class LocalSceneData &data );
		void initOpenGL( void );
		void updateBoundingBox( void );
		double intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , Util::BoundingBox1D range = Util::BoundingBox1D( Util::Epsilon , Util::Infinity ) , std::function< bool (double) > validityFunction = [] ( double t ){ return t>0; } ) const;
		bool isInside( Util::Point3D p ) const;
		void drawOpenGL( GLSLProgram * glslProgram ) const;
	};
}
#endif // RAY_FILE_INSTANCE_INCLUDED

