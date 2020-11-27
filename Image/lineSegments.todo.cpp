#include "lineSegments.h"
#include <math.h>
#include <Util/exceptions.h>

using namespace Util;
using namespace Image;

////////////////////////////
// Image processing stuff //
////////////////////////////
double OrientedLineSegment::length( void ) const
{
	return getLength(this->vector());
}

double OrientedLineSegment::distance( Point2D p ) const
{
	Point2D vectorSP = vector(this->endPoints[0], p);
	Point2D vectorST = this->vector();
	Point2D vectorSTPerp = this->perpendicular();
	Point2D vectorTP = vector(this->endPoints[1], p);

	double v = vectorSP.dot(vectorSTPerp) / getLength(vectorSTPerp);
	double u = vectorSP.dot(vectorST) / pow(getLength(vectorST), 2);

	if (u < 0)
		return getLength(vectorSP);
	else if (u > 1)
		return getLength(vectorTP);
	else
		return fabs(v);
}

Point2D OrientedLineSegment::perpendicular( void ) const
{
	Point2D result;
	result = rotateVector(this->vector(), 90);
	return normalizeVector(result);
}

Point2D OrientedLineSegment::GetSourcePosition( const OrientedLineSegment& source , const OrientedLineSegment& destination , Point2D target )
{
	Point2D vectorSPDest = vector(destination.endPoints[0], target);
	Point2D vectorSTDest = destination.vector();
	Point2D vectorSTPerpDest = destination.perpendicular();

	double v = vectorSPDest.dot(vectorSTPerpDest) / getLength(vectorSTPerpDest);
	double u = vectorSPDest.dot(vectorSTDest) / pow(getLength(vectorSTDest), 2);

	Point2D src = source.endPoints[0];
	src = src + (source.vector() * u);
	src = src + (source.perpendicular() * v);

	return src;
}

Point2D OrientedLineSegment::rotateVector(Point2D v, double angle)
{
	double angleRadian = angle * Pi / 180;
	double x = v[0] * cos(angleRadian) - v[1] * sin(angleRadian);
	double y = v[0] * sin(angleRadian) + v[1] * cos(angleRadian);
	return Point2D(x, y);
}

Point2D OrientedLineSegment::normalizeVector(Point2D v)
{
	double length = getLength(v);
	return Point2D(v[0] / length, v[1] / length);
}

double OrientedLineSegment::getLength(Point2D v)
{
	return sqrt(pow(v[0], 2) + pow(v[1], 2));
}

Point2D OrientedLineSegment::vector(void) const
{
	double x1 = this->endPoints[0][0];
	double y1 = this->endPoints[0][1];
	double x2 = this->endPoints[1][0];
	double y2 = this->endPoints[1][1];
	return Point2D(x2 - x1, y2 - y1);
}

Point2D OrientedLineSegment::vector(Point2D p1, Point2D p2)
{
	return Point2D(p2[0] - p1[0], p2[1] - p1[1]);
}