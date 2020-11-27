#include <stdio.h>
#include "fileInstance.h"

using namespace Ray;
using namespace Util;

//////////////////
// FileInstance //
//////////////////

FileInstance::FileInstance( void ) : _file(NULL) {}

void FileInstance::_write( std::ostream &stream ) const
{
	Shape::WriteInset( stream );
	stream << "#" << Directive() << "  " << _fileIndex;
}

void FileInstance::_read( std::istream &stream )
{
	if( !( stream >> _fileIndex ) ) THROW( "failed to read ray file index" );
}

void FileInstance::init( const LocalSceneData &data )
{
	// Set the .ray file pointer
	if( _fileIndex==-1 ) THROW( "negative ray_file index: %d" , _fileIndex );
	if( _fileIndex>=data.files.size() ) THROW( "shape specifies a ray_file that is out of bounds: %d <= %d" , _fileIndex , (int)data.files.size() );
	_file = &data.files[ _fileIndex ];
}

void FileInstance::updateBoundingBox( void ){ _bBox = _file->boundingBox(); }

void FileInstance::initOpenGL( void ){}

double FileInstance::intersect( Ray3D ray , RayShapeIntersectionInfo& iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const { return _file->intersect( ray , iInfo , range , validityLambda ); }

bool FileInstance::isInside( Point3D p ) const { return _file->isInside(p); }

void FileInstance::drawOpenGL( GLSLProgram * glslProgram ) const { _file->drawOpenGL( glslProgram ); }
