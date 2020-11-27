#include <Util/exceptions.h>
#include "triangle.h"
#include "shapeList.h"
#include "scene.h"

using namespace std;
using namespace Ray;
using namespace Util;

/////////////////
// AffineShape //
/////////////////
AffineShape::AffineShape( void ) : _shape(NULL){}

void AffineShape::initOpenGL( void ){ _shape->initOpenGL(); }

///////////////////////
// StaticAffineShape //
///////////////////////
StaticAffineShape::StaticAffineShape( void ) : AffineShape() , _localTransform( Matrix4D::Identity() ){}

void StaticAffineShape::set( Matrix4D m ){ _localTransform = m; }

void StaticAffineShape::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << "  " << _localTransform << std::endl;
	Shape::WriteInsetSize++;
	stream << *_shape;
	Shape::WriteInsetSize--;
}

void StaticAffineShape::_read( std::istream &stream )
{
	if( !( stream >> _localTransform ) ) THROW( "Failed to parse %s" , Directive().c_str() );
	if( _shape ) delete _shape;
	_shape = ReadShape( stream , ShapeList::ShapeFactories );
}

void StaticAffineShape::init( const LocalSceneData &data )
{
	_inverseTransform = _localTransform.inverse();
	_normalTransform = _localTransform.inverse().transpose();
	_shape->init( data );
}

void StaticAffineShape::initOpenGL( void ){ _shape->initOpenGL(); }

Matrix4D StaticAffineShape::getMatrix( void ) const { return _localTransform; }

Matrix4D StaticAffineShape::getInverseMatrix( void ) const { return _inverseTransform; }

Matrix3D StaticAffineShape::getNormalMatrix( void ) const{ return _normalTransform; }

////////////////////////
// DynamicAffineShape //
////////////////////////
DynamicAffineShape::DynamicAffineShape( void ) : AffineShape() , _matrix(NULL) {}

void DynamicAffineShape::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << "  " << _paramName << std::endl;
	Shape::WriteInsetSize++;
	stream << *_shape;
	Shape::WriteInsetSize--;
}

void DynamicAffineShape::_read( std::istream &stream )
{
	if( !( stream >> _paramName ) ) THROW( "Failed to parse %s" , Directive().c_str() );
	if( _shape ) delete _shape;
	_shape = ReadShape( stream , ShapeList::ShapeFactories );
}

void DynamicAffineShape::init( const LocalSceneData &data )
{
	if( !data.keyFrameFile ) THROW( "no key-frame file" );
	_matrix = &data.keyFrameFile->keyFrameMatrices.current( _paramName );
	_shape->init( data );
}

Matrix4D DynamicAffineShape::getMatrix( void ) const { return *_matrix; }

Matrix4D DynamicAffineShape::getInverseMatrix( void ) const { return getMatrix().inverse(); }

Matrix3D DynamicAffineShape::getNormalMatrix( void ) const { return getMatrix().inverse().transpose(); }

////////////////
// Difference //
////////////////
Difference::Difference( Shape *shape0 , Shape *shape1 ) : _shape0(shape0) , _shape1(shape1) {}

void Difference::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << std::endl;
	Shape::WriteInsetSize++;
	stream << *_shape0 << std::endl;
	stream << *_shape1;
	Shape::WriteInsetSize--;
}

void Difference::_read( std::istream &stream )
{
	string keyword;

	try{ keyword = ReadDirective( stream ); }
	catch( Util::Exception e ){ THROW( "failed to read directive in %s\n%s" , name().c_str() , e.what() ); }
	if( ShapeList::ShapeFactories.find( keyword )!=ShapeList::ShapeFactories.end() )
	{
		_shape0 = ShapeList::ShapeFactories[ keyword ]->create();
		if( !_shape0 ) THROW( "failed to allocate memory for %s" , keyword.c_str() );
		stream >> *_shape0;
	}

	try{ keyword = ReadDirective( stream ); }
	catch( Util::Exception e ){ THROW( "failed to read directive in %s\n%s" , name().c_str() , e.what() ); }
	if( ShapeList::ShapeFactories.find( keyword )!=ShapeList::ShapeFactories.end() )
	{
		_shape1 = ShapeList::ShapeFactories[ keyword ]->create();
		if( !_shape1 ) THROW( "failed to allocate memory for %s" , keyword.c_str() );
		stream >> *_shape1;
	}
	else THROW( "unexpected directive in group %s: %s" , name().c_str() , keyword.c_str() );
}

void Difference::init( const LocalSceneData& data ){ _shape0->init( data ) , _shape1->init( data ); }

void Difference::initOpenGL( void )
{
	THROW( "OpenGL rendering not supported for %s" , name().c_str() );
}

void Difference::drawOpenGL( GLSLProgram *glslProgram ) const
{
	THROW( "OpenGL rendering not supported for %s" , name().c_str() );
}

/////////////////////////
// ShapeBoundingBoxHit //
/////////////////////////
bool ShapeBoundingBoxHit::Compare(const ShapeBoundingBoxHit& v1, const ShapeBoundingBoxHit& v2) { return v1.t < v2.t; }

///////////////
// ShapeList //
///////////////
std::unordered_map< std::string , BaseFactory< Shape > * > ShapeList::ShapeFactories;

void ShapeList::_read( std::istream &stream )
{
	string endDirective = _DirectiveHeader() + string( "_end" );
	while( true )
	{
		string keyword;
		try{ keyword = ReadDirective( stream ); }
		catch( Util::Exception e ){ THROW( "failed to read directive in %s\n%s" , name().c_str() , e.what() ); }
		// Test if we are closing the list
		if( keyword==endDirective ) return;
		// Otherwise read the next shape
		else if( ShapeList::ShapeFactories.find( keyword )!=ShapeList::ShapeFactories.end() )
		{
			Shape *shape = ShapeList::ShapeFactories[ keyword ]->create();
			if( !shape ) THROW( "failed to allocate memory for %s" , keyword.c_str() );
			stream >> *shape;
			shapes.push_back( shape );
		}
		else THROW( "unexpected directive in group %s: %s" , name().c_str() , keyword.c_str() );
	}
}

void ShapeList::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << std::endl;
	Shape::WriteInsetSize++;
	for( int i=0 ; i<shapes.size() ; i++ ) stream << *shapes[i] << std::endl;
	Shape::WriteInsetSize--;
	Shape::WriteInset( stream );
	stream << "#" << _DirectiveHeader() << "_end";
}

void ShapeList::addTrianglesOpenGL( std::vector< TriangleIndex >& triangles )
{
	for( int i=0 ; i<shapes.size() ; i++ ) shapes[i]->addTrianglesOpenGL( triangles );
}

//////////////////
// TriangleList //
//////////////////
TriangleList::TriangleList( void ) : _vertices(NULL) , _vNum(0) , _vertexBufferID(0) , _elementBufferID(0){}

void TriangleList::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << "  " << _materialIndex << std::endl;
	Shape::WriteInsetSize++;
	stream << _shapeList;
	Shape::WriteInsetSize--;
}

void TriangleList::_read( std::istream &stream )
{
	if( !( stream >> _materialIndex ) ) THROW( "failed to read material index for %s" , name().c_str() );
	string keyword = ReadDirective( stream );
	if( keyword!=ShapeList::Directive() ) THROW( "%s expects next shape to be: %s" , name().c_str() , ShapeList::Directive().c_str() );
	stream >> _shapeList;
}

void TriangleList::updateBoundingBox( void )
{
	_shapeList.updateBoundingBox();
	_bBox = _shapeList.boundingBox();
}

bool TriangleList::isInside( Point3D p ) const { return _shapeList.isInside( p ); }

void TriangleList::addTrianglesOpenGL( std::vector< TriangleIndex > &triangles ){ _shapeList.addTrianglesOpenGL( triangles ); }

///////////
// Union //
///////////
void Union::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << std::endl;
	Shape::WriteInsetSize++;
	stream << _shapeList;
	Shape::WriteInsetSize--;
}

void Union::_read( std::istream &stream )
{
	string keyword = ReadDirective( stream );
	if( keyword!=ShapeList::Directive() ) THROW( "%s expects next shape to be: %s" , name().c_str() , ShapeList::Directive().c_str() );
	stream >> _shapeList;
}

void Union::drawOpenGL( GLSLProgram *glslProgram ) const
{
	THROW( "OpenGL rendering not supported for %s" , name().c_str() );
}

void Union::initOpenGL( void )
{
	THROW( "OpenGL rendering not supported for %s" , name().c_str() );
}

//////////////////
// Intersection //
//////////////////
void Intersection::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << std::endl;
	Shape::WriteInsetSize++;
	stream << _shapeList;
	Shape::WriteInsetSize--;
}

void Intersection::_read( std::istream &stream )
{
	string keyword = ReadDirective( stream );
	if( keyword!=ShapeList::Directive() ) THROW( "%s expects next shape to be: %s" , name().c_str() , ShapeList::Directive().c_str() );
	stream >> _shapeList;
}

void Intersection::drawOpenGL( GLSLProgram *glslProgram ) const
{
	THROW( "OpenGL rendering not supported for %s" , name().c_str() );
}

void Intersection::initOpenGL( void )
{
	THROW( "OpenGL rendering not supported for %s" , name().c_str() );
}
