/*
Copyright (c) 2019, Michael Kazhdan
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <algorithm>
#include <SVD/SVDFit.h>
#include <SVD/MatrixMNTC.h>
#include <Util/exceptions.h>

namespace Util
{
	///////////
	// Point //
	///////////
	template< unsigned int Dim >
	void Point< Dim >::_init( const double *values , unsigned int sz )
	{
		if     ( sz==0   ) memset( _p , 0 , sizeof(_p) );
		else if( sz==Dim ) memcpy( _p , values , sizeof(_p) );
		else ERROR_OUT( "Should never be called" );
	}

	template< unsigned int Dim >
	Point< Dim >::Point( void ){ memset( _p , 0 , sizeof(_p) ); }

	template< unsigned int Dim >
	Point< Dim >::Point( const Point &p ){ memcpy( _p , p._p , sizeof(_p) ); }

	template< unsigned int Dim >
	template< typename ... Doubles >
	Point< Dim >::Point( Doubles ... values )
	{
		static_assert( sizeof...(values)==Dim || sizeof...(values)==0 , "[ERROR] Point< Dim >::Point: Invalid number of coefficients" );
		const double _values[] = { values... };
		_init( _values , sizeof...(values) );
	}

	template< unsigned int Dim >
	Point< Dim > Point< Dim >::operator * ( double s ) const { Point p ; for( int i=0 ; i<Dim ; i++ ) p._p[i] = _p[i] * s ; return p; }

	template< unsigned int Dim >
	Point< Dim > Point< Dim >::operator + ( const Point &p ) const { Point q ; for( int i=0 ; i<Dim ; i++ ) q._p[i] += _p[i] + p._p[i] ; return q; }

	template< unsigned int Dim >
	double Point< Dim >::dot( const Point &q ) const
	{
		double dot = 0;
		for( int i=0 ; i<Dim ; i++ ) dot += _p[i] * q._p[i];
		return dot;
	}

	template< unsigned int Dim >
	double& Point< Dim >::operator[] ( int i ){ return _p[i]; }

	template< unsigned int Dim >
	const double &Point< Dim >::operator[] ( int i ) const { return _p[i]; }

	template< unsigned int Dim >
	Point< Dim >  Point< Dim >::operator * ( const Point &q ) const
	{
		Point p;
		for( int i=0 ; i<Dim ; i++ ) p[i] = _p[i]*q._p[i];
		return p;
	}

	template< unsigned int Dim >
	Point< Dim >  Point< Dim >::operator / ( const Point &q ) const
	{
		Point p;
		for( int i=0 ; i<Dim ; i++ ) p[i] = _p[i]/q._p[i];
		return p;
	}

	template< unsigned int Dim >
	Point< Dim > &Point< Dim >::operator *= ( const Point &q ){	return (*this) = (*this) * q; }

	template< unsigned int Dim >
	Point< Dim > &Point< Dim >::operator /= ( const Point &q ){	return (*this) = (*this) / q; }

	template< unsigned int Dim >
	template< typename ... Points >
	Point< Dim > Point< Dim >::CrossProduct( Points ... points )
	{
		static_assert( sizeof ... ( points )==Dim-1 , "[ERROR] Number of points in cross-product must be one less than the dimension" );
		const Point< Dim > _points[] = { points ... };
		return CrossProduct( _points );
	}

	template< unsigned int Dim >
	Point< Dim > Point< Dim >::CrossProduct( Point *points ){ return CrossProduct( (const Point *)points );}

	template< unsigned int Dim >
	Point< Dim > Point< Dim >::CrossProduct( const Point *points )
	{
		Matrix< Dim > M;
		for( int d=0 ; d<Dim ; d++ ) for( int c=0 ; c<Dim-1 ; c++ ) M(d,c) = points[c][d];
		Point p;
		for( int d=0 ; d<Dim ; d++ ) p[d] = ( d&1 ) ? -M.subDeterminant( d , Dim-1 ) : M.subDeterminant( d , Dim-1 );
		return p;
	}

	template< unsigned int Dim >
	std::ostream &operator << ( std::ostream &stream , const Point< Dim > &p )
	{
		for( int i=0 ; i<Dim-1 ; i++ ) stream << p[i] << " ";
		stream << p[Dim-1];
		return stream;
	}
	template< unsigned int Dim >
	std::istream &operator >> ( std::istream &stream , Point< Dim > &p )
	{
		for( int i=0 ; i<Dim ; i++ ) stream >> p[i];
		return stream;
	}

	/* Additional methods */

	template <unsigned int Dim>
	Point<Dim> Point<Dim>::normalize(void) const
	{
		double magnitude = sqrt(this->dot(*this));
		return (*this / magnitude);
	}

	template <unsigned int Dim>
	double Point<Dim>::magnitude(void) const
	{
		return sqrt(this->dot(*this));
	}

	template <unsigned int Dim>
	bool Point<Dim>::operator == (const Point<Dim>& p) const
	{
		for (int i = 0; i < Dim; i++)
			if (this->_p[i] != p._p[i])
				return false;
		return true;
	}

	////////////
	// Matrix //
	////////////
	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::operator * ( double s ) const { Matrix n ; for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) n._m[i][j] = _m[i][j] * s ; return n; }

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::operator + ( const Matrix &m ) const { Matrix n ; for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) n._m[i][j] += _m[i][j] + m._m[i][j] ; return n; }

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::operator * ( const Matrix &m ) const
	{
		Matrix n;
		for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) for( int k=0 ; k<Dim ; k++ ) n._m[i][j] += _m[k][j] * m._m[i][k];
		return n;
	}

	template< unsigned int Dim >
	double Matrix< Dim >::dot( const Matrix &m ) const
	{
		double dot = 0;
		for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) dot += _m[i][j] * m._m[i][j];
		return dot;
	}

	template< unsigned int Dim >
	Matrix< Dim >::Matrix( void ){ memset( _m , 0 , sizeof(_m) ); }

	template< unsigned int Dim >
	Matrix< Dim >::Matrix( const Matrix< Dim+1 > &n ){ for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) _m[i][j] = n[i][j]; }

	template< unsigned int Dim >
	Matrix< Dim >::Matrix( const Matrix< Dim-1 > &n , Point< Dim-1 > p ) : Matrix()
	{
		for( int i=0 ; i<Dim-1 ; i++ ) for( int j=0 ; j<Dim-1 ; j++ ) _m[i][j] = n[i][j];
		_m[Dim-1][Dim-1] = 1.;
		for( int i=0 ; i<Dim-1 ; i++ ) _m[Dim-1][i] = p[i];
	}

	template< unsigned int Dim >
	double *Matrix< Dim >::operator[] ( int c ) { return _m[c]; }

	template< unsigned int Dim >
	const double *Matrix< Dim >::operator[] ( int c ) const { return _m[c]; }

	template< unsigned int Dim >
	double& Matrix< Dim >::operator() ( int r , int c )       { return _m[c][r]; }

	template< unsigned int Dim >
	const double &Matrix< Dim >::operator() ( int r , int c ) const { return _m[c][r]; }

	template< unsigned int Dim >
	double Matrix< Dim >::subDeterminant( int r , int c ) const
	{
		Matrix< Dim-1 > m;
		int rr[Dim-1] , cc[Dim-1];
		for( int a=0 , _r=0 , _c=0 ; a<Dim ; a++ )
		{
			if( a!=r ) rr[_r++] = a;
			if( a!=c ) cc[_c++] = a;
		}
		for( int _c=0 ; _c<Dim-1 ; _c++ ) for( int _r=0 ; _r<Dim-1 ; _r++ ) m(_r,_c) = _m[ cc[_c] ][ rr[_r] ];
		return m.determinant();
	}

	template< unsigned int Dim >
	double Matrix< Dim >::determinant( void ) const
	{
		double det = 0.;
		for( int d=0 ; d<Dim ; d++ ) 
			if( d&1 ) det -= _m[d][0] * subDeterminant( 0 , d );
			else      det += _m[d][0] * subDeterminant( 0 , d );
		return det;
	}

	template<>
	inline double Matrix< 1 >::determinant( void ) const { return _m[0][0]; }

	template< unsigned int Dim >
	double Matrix< Dim >::trace( void ) const
	{
		double tr = 0;
		for( int i=0 ; i<Dim ; i++ ) tr += _m[i][i];
		return tr;
	}

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::transpose( void ) const
	{
		Matrix n;
		for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) n._m[i][j] = _m[j][i];
		return n;
	}

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::inverse( void ) const
	{
		Matrix inv;
		double d = determinant();
		if( !d ) THROW( "singular matrix" );
		for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ )
			if( (i+j)%2==0 ) inv._m[j][i] =  subDeterminant( j , i ) / d;
			else             inv._m[j][i] = -subDeterminant( j , i ) / d;
		return inv;
	}

	template<>
	inline Matrix< 1 > Matrix< 1 >::inverse( void ) const
	{
		Matrix< 1 > m;
		m._m[0][0] = 1./_m[0][0];
		return m;
	}

	template< unsigned int Dim >
	Point< Dim > Matrix< Dim >::operator * ( const Point< Dim > &p ) const
	{
		Point< Dim > q;
		for( int i=0 ; i<Dim ; i++ ) for( int j=0 ; j<Dim ; j++ ) q[j] += _m[i][j] * p[i];
		return q;
	}

	template< unsigned int Dim >
	Point< Dim-1 > Matrix< Dim >::operator * ( const Point< Dim-1 > &p ) const
	{
		Point< Dim > q;
		for( int i=0 ; i<Dim-1 ; i++ ) q[i] = p[i];
		q[Dim-1] = 1;
		q = (*this) * q;
		Point< Dim-1 > _q;
		for( int i=0 ; i<Dim-1 ; i++ ) _q[i] = q[i] / q[Dim-1];
		return _q;
	}

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::Identity( void )
	{
		Matrix m;
		for( int i=0 ; i<Dim ; i++ ) m[i][i] = 1;
		return m;
	}

	template< unsigned int Dim >
	void Matrix< Dim >::SVD( Matrix& r1 , Matrix& d , Matrix& r2 ) const
	{
		GXMatrixMNd M( Dim , Dim );
		GXMatrixMNd U, W, Vt;

		d = r1 = r2 = Identity();

		for( int i=0 ; i<Dim ; i++ ) for(  int j=0 ; j<Dim ; j++ ) M(i,j) = _m[j][i];
		SVDMat( M , U , W , Vt );  // M == U . DiagonalMatrix(W) . Vt
		for( int i=0 ; i<Dim ; i++ )
		{
			for( int j=0 ; j<Dim ; j++ ) r1[i][j] = U(j,i) , r2[i][j] = Vt(j,i);
			d[i][i] = W(i,0);
		}
	}

	// Code borrowed from:
	// Linear Combination of Transformations
	// Marc Alexa
	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::SquareRoot( const Matrix& m , double eps )
	{
		Matrix X,Y;
		X = m;
		Y = Identity();
		while( (X*X-m).squareNorm()>eps*eps )
		{
			Matrix iX = X.inverse();
			Matrix iY = Y.inverse();
			X = (X+iY) / 2;
			Y = (Y+iX) / 2;
		}
		return X;
	}

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::Log( const Matrix& m , double eps )
	{
		Matrix I = Identity();
		Matrix X , Z , A=m;
		int k=0;
		while( (A-I).squareNorm()>0.25 )
		{
			A = SquareRoot( A , eps );
			k++;
		}
		A = I-A;
		X = Z = A;
		int i = 1;
		while( Z.squareNorm()>eps*eps )
		{
			Z = Z*A;
			i++;
			X += Z*( 1.0/i );
		}
		return X * ( -pow( 2.0 , (double)k ) );
	}

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::symmetrize( void ) const { return ( (*this)+transpose() ) / 2; }

	template< unsigned int Dim >
	Matrix< Dim > Matrix< Dim >::skewSymmetrize( void ) const { return ( (*this)-transpose() ) / 2; }

	template< unsigned int Dim >
	std::ostream &operator << ( std::ostream &stream , const Matrix< Dim > &m )
	{
		const double *_m = m[0];
		for( int i=0 ; i<Dim*Dim-1 ; i++ ) stream << _m[i] << " ";
		stream << _m[Dim*Dim-1];
		return stream;
	}

	template< unsigned int Dim >
	std::istream &operator >> ( std::istream &stream , Matrix< Dim > &m )
	{
		double *_m = &m(0,0);
		for( int i=0 ; i<Dim*Dim ; i++ ) stream >> _m[i];
		return stream;
	}

	///////////
	// Plane //
	///////////
	template< unsigned int Dim >
	Plane< Dim >::Plane( void ) : distance(0) {}

	template< unsigned int Dim >
	Plane< Dim >::Plane( const Point< Dim > &n , const Point< Dim > &p )
	{
		normal = n.unit();
		distance = Point< Dim >::Dot( normal , p );
	}

	template< unsigned int Dim >
	template< typename ... Points >
	Plane< Dim >::Plane( Points ... points )
	{
		static_assert( sizeof ... ( points )==Dim , "[ERROR] Number of points in plane constructor must equal the dimension" );
		const Point< Dim > _points[] = { points ... };
		(*this) = Plane( _points );
	}

	template< unsigned int Dim >
	Plane< Dim >::Plane( Point< Dim > *points ) : Plane( (const Point< Dim > *)points ) {}

	template< unsigned int Dim >
	Plane< Dim >::Plane( const Point< Dim > *points )
	{
		Point< Dim > _points[Dim-1];
		for( int i=1 ; i<Dim ; i++ ) _points[i-1] = points[i] - points[0];
		normal = Point< Dim >::CrossProduct( _points ).unit();
		distance = Point< Dim >::Dot( normal , points[0] );
	}

	template< unsigned int Dim >
	double Plane< Dim >::operator() ( const Point< Dim > &p ) const
	{
		return Point< Dim >::Dot( normal , p ) + distance;
	}

	/////////
	// Ray //
	/////////
	template< unsigned int Dim >
	Ray< Dim >::Ray( void ){}

	template< unsigned int Dim >
	Ray< Dim >::Ray( const Point< Dim > &p , const Point< Dim > &d ) : position(p) , direction(d) {}

	template< unsigned int Dim >
	Point< Dim > Ray< Dim >::operator() ( double s ) const { return position+direction*s; }

	template< unsigned int Dim >
	Ray< Dim >  Ray< Dim >::operator +  ( const Point< Dim > &p ) const { return Ray( position+p , direction );}

	template< unsigned int Dim >
	Ray< Dim > &Ray< Dim >::operator += ( const Point< Dim > &p ){ position += p ; return *this; }

	template< unsigned int Dim >
	Ray< Dim >  Ray< Dim >::operator -  ( const Point< Dim > &p ) const { return Ray( position-p , direction );}

	template< unsigned int Dim >
	Ray< Dim > &Ray< Dim >::operator -= ( const Point< Dim > &p ){ position -= p ; return *this; }

	template< unsigned int Dim >
	Ray< Dim > operator * ( const Matrix< Dim+1 > &m , const Ray< Dim >& r )
	{
		return Ray< Dim >( m * r.position , Matrix< Dim >(m) * r.direction );
	}


	/////////////////
	// BoundingBox //
	/////////////////
	template< unsigned int Dim >
	BoundingBox< Dim >::BoundingBox( void ){}

	template< unsigned int Dim >
	BoundingBox< Dim >::BoundingBox( const Point< Dim > &p1 , const Point< Dim > &p2 )
	{
		for( int d=0 ; d<Dim ; d++ ) _p[0][d] = std::min< double >( p1[d] , p2[d] ) , _p[1][d] = std::max< double >( p1[d] , p2[d] );
	}

	template< unsigned int Dim >
	BoundingBox< Dim >::BoundingBox( const Point< Dim > *pList , int pSize )
	{
		if( pSize>0 )
		{
			_p[0] = _p[1] = pList[0];
			for( int i=1 ; i<pSize ; i++ ) for( int j=0 ; j<Dim ; j++ ) _p[0][j] = std::min< double >( _p[0][j] , pList[i][j] ) , _p[1][j] = std::max< double >( _p[1][j] , pList[i][j] );
		}
	}

	template< unsigned int Dim >
	Point< Dim > &BoundingBox< Dim >::operator[] ( int idx ){ return _p[idx]; }

	template< unsigned int Dim >
	const Point< Dim > &BoundingBox< Dim >::operator[] ( int idx ) const { return _p[idx]; }

	template< unsigned int Dim >
	BoundingBox< Dim > BoundingBox< Dim >::operator + ( const BoundingBox &b ) const
	{
		Point< Dim > pList[4];
		Point< Dim > q;

		if( b.isEmpty() ) return *this;
		if(   isEmpty() ) return b;
		pList[0] = _p[0];
		pList[1] = _p[1];
		pList[2] = b._p[0];
		pList[3] = b._p[1];
		return BoundingBox( pList , 4 );
	}

	template< unsigned int Dim >
	BoundingBox< Dim > &BoundingBox< Dim >::operator += ( const BoundingBox &b ){ return (*this) = (*this) + b; }

	template< unsigned int Dim >
	BoundingBox< Dim > BoundingBox< Dim >::operator ^ ( const BoundingBox &b ) const
	{

		if( isEmpty() || b.isEmpty() ) return BoundingBox();
		BoundingBox _b;
		for( int j=0 ; j<Dim ; j++ ) _b._p[0][j] = std::max< double >( _p[0][j] , b._p[0][j] ) , _b._p[1][j] = std::min< double >( _p[1][j] , b._p[1][j] );
		if( _b.isEmpty() ) _b._p[0] = _b._p[1] = ( _b._p[0] + _b._p[1] ) / 2;
		return _b;
	}

	template< unsigned int Dim >
	BoundingBox< Dim > &BoundingBox< Dim >::operator ^= ( const BoundingBox &b ){ return (*this) = (*this) ^ b; }

	template< unsigned int Dim >
	BoundingBox< Dim > operator * ( const Matrix< Dim+1 > &m , const BoundingBox< Dim > &b )
	{
		Point< Dim > v[1<<Dim];
		for( int idx=0 ; idx<(1<<Dim) ; idx++ )
		{
			Point< Dim > p;
			for( int d=0 ; d<Dim ; d++ ) p[d] = b[(idx>>d)&1][d];
			v[idx] = m * p;
		}
		return BoundingBox< Dim >( v , 1<<Dim );
	}

	template< unsigned int Dim >
	bool BoundingBox< Dim >::isInside( const Point< Dim > &p ) const
	{
		for( int d=0 ; d<Dim ; d++ ) if( p[d]<=_p[0][d] || p[d]>=_p[1][d] ) return false;
		return true;
	}

	template< unsigned int Dim >
	bool BoundingBox< Dim >::isInsideOrOnBoundary(const Point< Dim >& p) const
	{
		for (int d = 0; d < Dim; d++) if (p[d] < _p[0][d] || p[d] > _p[1][d]) return false;
		return true;
	}

	template< unsigned int Dim >
	bool BoundingBox< Dim >::isEmpty( void ) const
	{
		for( int d=0 ; d<Dim ; d++ ) if( _p[0][d]>=_p[1][d] ) return true;
		return false;
	}

	template< unsigned int Dim >
	std::ostream &operator << ( std::ostream &stream , const BoundingBox< Dim > &b )
	{
		stream << "[ " << b[0] << " ] [ " << b[1] << " ]";
		return stream;
	}

	///////////////////////
	// RotationParameter //
	///////////////////////
	template< typename RotationParameterType , typename ParameterType >
	RotationParameterType RotationParameter< RotationParameterType , ParameterType >::operator * ( double scale ) const
	{
		RotationParameterType p;
		p.parameter = parameter * scale;
		return p;
	}

	template< typename RotationParameterType , typename ParameterType >
	RotationParameterType RotationParameter< RotationParameterType , ParameterType >::operator + ( const RotationParameterType &p ) const
	{
		RotationParameterType _p;
		_p.parameter = parameter + p.parameter;
		return _p;
	}

	/////////////////////////////
	// TransformationParameter //
	/////////////////////////////
	template< typename RotationParameterType >
	TransformationParameter< RotationParameterType > TransformationParameter< RotationParameterType >::operator * ( double scale ) const
	{
		TransformationParameter tp = *this;
		tp.rotationParameter *= scale;
		tp.translation *= scale;
		return tp;
	}

	template< typename RotationParameterType >
	TransformationParameter< RotationParameterType > TransformationParameter< RotationParameterType >::operator + ( const TransformationParameter &tp ) const
	{
		TransformationParameter _tp = *this;
		_tp.rotationParameter += tp.rotationParameter;
		_tp.translation += tp.translation;
		return _tp;
	}

	template< typename RotationParameterType >
	TransformationParameter< RotationParameterType >::TransformationParameter( void ) {}

	template< typename RotationParameterType >
	TransformationParameter< RotationParameterType >::TransformationParameter( const Matrix4D &m ) : rotationParameter( Matrix3D( m ) )
	{
		for( int i=0 ; i<3 ; i++ ) translation[i] = m[3][i];
	}

	template< typename RotationParameterType >
	TransformationParameter< RotationParameterType >::TransformationParameter( const Matrix4D &m , const TransformationParameter &p ) : rotationParameter( Matrix3D( m ) , p.rotationParameter )
	{
		for( int i=0 ; i<3 ; i++ ) translation[i] = m[3][i];
	}

	template< typename RotationParameterType >
	Matrix4D TransformationParameter< RotationParameterType >::operator () ( void ) const
	{
		return Matrix4D( rotationParameter() , translation );
	}


	/** Helper functions */

	template<typename T>
	T max(T a, T b)
	{
		if (a >= b)
			return a;
		else
			return b;
	}

	template<typename T>
	T min(T a, T b)
	{
		if (a >= b)
			return b;
		else
			return a;
	}
}