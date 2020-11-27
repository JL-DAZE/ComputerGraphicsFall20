#include "shape.h"

using namespace Ray;
using namespace Util;

///////////
// Shape //
///////////
unsigned int Shape::OpenGLTessellationComplexity;

unsigned int Shape::WriteInsetSize = 0;

void Shape::WriteInset( std::ostream &stream ){ for( unsigned int i=0 ; i<WriteInsetSize ; i++ ) stream << "  "; }

BoundingBox3D Shape::boundingBox( void ) const { return _bBox; }
