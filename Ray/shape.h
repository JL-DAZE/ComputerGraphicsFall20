#ifndef SHAPE_INCLUDED
#define SHAPE_INCLUDED
#include <vector>
#include <stdexcept>
#include <string>
#include <functional>
#include <Util/geometry.h>
#include <Util/factory.h>
#include <GL/glew.h>
#pragma warning( disable : 4290 )
#ifdef __APPLE__
#include <GLUT/glut.h>
#else // !__APPLE__
#include <GL/glut.h>
#endif // __APPLE__
#include <Util/exceptions.h>
#include "GLSLProgram.h"

#ifdef VERBOSE_MESSAGING
inline void AssertOpenGLState( const char *fileName , int line , const char *functionName )
{
	GLenum errorCode;
	if( ( errorCode=glGetError() )!=GL_NO_ERROR )
	{
		char *buffer = Util::MakeMessageString( "[OPEN_GL ERROR]" , fileName , line , functionName , "%s (%d)" , gluErrorString( errorCode ) , errorCode );
		std::cerr << buffer << std::endl;
		delete[] buffer;
		exit( 0 );
	}
}
#ifndef ASSERT_OPEN_GL_STATE
#define ASSERT_OPEN_GL_STATE( ... ) AssertOpenGLState( __FILE__ , __LINE__ , __FUNCTION__ )
#endif // ASSERT_OPEN_GL_STATE
#else // !VERBOSE_MESSAGING
inline void AssertOpenGLState( const char *functionName )
{
	GLenum errorCode;
	if( ( errorCode=glGetError() )!=GL_NO_ERROR )
	{
		char *buffer = Util::MakeMessageString( "[OPEN_GL ERROR]" , functionName , "%s (%d)" , gluErrorString( errorCode ) , errorCode );
		std::cerr << buffer << std::endl;
		delete[] buffer;
		exit( 0 );
	}
}
#ifndef ASSERT_OPEN_GL_STATE
#define ASSERT_OPEN_GL_STATE( ... ) AssertOpenGLState( __FUNCTION__ )
#endif // ASSERT_OPEN_GL_STATE
#endif // VERBOSE_MESSAGING


namespace Ray
{
	/** This is the abstract class that all ray-traceable objects must implement. */
	class Shape
	{
		friend std::ostream &operator << ( std::ostream & , const Shape & );
		friend std::istream &operator >> ( std::istream & ,       Shape & );

		/** This method writes the Shape into the stream (including the starting directive) */
		virtual void _write( std::ostream &stream ) const = 0;

		/** This method reads the Shape from the stream (excluding the starting directive) */
		virtual void _read( std::istream &stream ) = 0;
	protected:
		/** This member represents the bounding box of the shape. */
		Util::BoundingBox3D _bBox;

	public:
		/** A global variable representing how finely shapes should be tessellated for rendering as triangle meshes. */
		static unsigned int OpenGLTessellationComplexity;

		/** A global variable representing how inset the next shape should be when writing to a stream. */
		static unsigned int WriteInsetSize;

		/** This static method insets for writing. */
		static void WriteInset( std::ostream &stream );

		/** The destructor */
		virtual ~Shape( void ){}

		/** The method returns the bounding box of the shape */
		Util::BoundingBox3D boundingBox( void ) const;

		/** This method should be called (once) immediately after setting up the scene graph */
		virtual void init( const class LocalSceneData& data ) = 0;

		/** This method should be called (once) after an OpenGL context has been created */
		virtual void initOpenGL( void ) = 0;

		/** This method should be called to update the bounding boxes in the scene */
		virtual void updateBoundingBox( void ) = 0;

		/** This method returns the name of the shape */
		virtual std::string name( void ) const = 0;

		/** This method computes the intersection of the shape with the ray.
		*** It should return the first value, t, within the prescribed range for which the validity lambda is true.
		*** If a valid intersection is found, it is returned and the intersection information is set.
		*** Otherwise a value of Infinity is returned.
		*** By default, the range is assumed to be (Epsilon,Infinity) and the validity function is a trivial function that returns true. */
		virtual double intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , Util::BoundingBox1D range = Util::BoundingBox1D( Util::Epsilon , Util::Infinity ) , std::function< bool (double) > validityLambda = [] ( double t ){ return true; }  ) const = 0;

		/** This method determines if a point is inside a shape.
		*** It is assumed that if the shape is not water-tight, the method returns false. */
		virtual bool isInside( Util::Point3D p ) const = 0;

		/** This method calls the necessary OpenGL commands to render the primitive. */
		virtual void drawOpenGL( GLSLProgram *glslProgram ) const=0;

		/** This method adds a triangle to the list of triangles (if the object is of type RayTriangle). */
		virtual void addTrianglesOpenGL( std::vector< class TriangleIndex > &triangles ) {}
	};

	/** This operator writes the shape out to a stream. */
	inline std::ostream &operator << ( std::ostream &stream , const Shape &shape ){ shape._write( stream ) ; return stream; }

	/** This operator reads a shape from a stream. */
	inline std::istream &operator >> ( std::istream &stream ,       Shape &shape ){ shape._read( stream ) ; return stream; }
}
#endif // SHAPE_INCLUDED
