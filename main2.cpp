#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <Util/cmdLineParser.h>
#include <Util/timer.h>
#include <Ray/scene.h>
#include <Ray/box.h>
#include <Ray/cone.h>
#include <Ray/cylinder.h>
#include <Ray/sphere.h>
#include <Ray/torus.h>
#include <Ray/triangle.h>
#include <Ray/fileInstance.h>
#include <Ray/directionalLight.h>
#include <Ray/pointLight.h>
#include <Ray/spotLight.h>

using namespace std;
using namespace Ray;
using namespace Util;
using namespace Image;

#undef GLUT_NO_LIB_PRAGMA

CmdLineParameter< string > InputRayFile( "in" );
CmdLineParameter< string > OutputImageFile( "out" );
CmdLineParameter< int > ImageWidth( "width" , 640 );
CmdLineParameter< int > ImageHeight( "height" , 480 );
CmdLineParameter< int > RecursionLimit( "rLimit" , 5 );
CmdLineParameter< float > CutOffThreshold( "cutOff" , 0.0001f );
// CmdLineParameter< string > SuperSampling( "SS" );

CmdLineReadable* params[] =
{
	&InputRayFile , &OutputImageFile , &ImageWidth , &ImageHeight , &RecursionLimit , &CutOffThreshold , /*&SuperSampling ,*/
	NULL
};

void ShowUsage( const string &ex )
{
	cout << "Usage " << ex << ":" << endl;
	cout << "\t --" << InputRayFile.name << " <input ray File>" << endl;
	cout << "\t[--" << OutputImageFile.name << " <output image file>]" << endl;
	cout << "\t[--" << ImageWidth.name << " <image width>=" << ImageWidth.value << "]" << endl;
	cout << "\t[--" << ImageHeight.name << " <image height>=" << ImageHeight.value << "]" << endl;
	cout << "\t[--" << RecursionLimit.name << " <recursion limit>=" << RecursionLimit.value << "]" << endl;
	cout << "\t[--" << CutOffThreshold.name << " <cut-off threshold>=" << CutOffThreshold.value << "]" << endl;
	// cout << "\t[--" << SuperSampling.name << " <Supersampling>]" << endl;
}

int main( int argc , char *argv[] )
{
	bool ss = false;
	CmdLineParse( argc-1 , argv+1 , params );
	if( !InputRayFile.set ){ ShowUsage( argv[0] ) ; return EXIT_FAILURE; }
	// if (SuperSampling.set) { ss = true; }

	Scene scene;
	try
	{
		ShapeList::ShapeFactories[ Box              ::Directive() ] = new DerivedFactory< Shape , Box >();
		ShapeList::ShapeFactories[ Cone             ::Directive() ] = new DerivedFactory< Shape , Cone >();
		ShapeList::ShapeFactories[ Cylinder         ::Directive() ] = new DerivedFactory< Shape , Cylinder >();
		ShapeList::ShapeFactories[ Sphere           ::Directive() ] = new DerivedFactory< Shape , Sphere >();
		ShapeList::ShapeFactories[ Torus            ::Directive() ] = new DerivedFactory< Shape , Torus >();
		ShapeList::ShapeFactories[ Triangle         ::Directive() ] = new DerivedFactory< Shape , Triangle >();
		ShapeList::ShapeFactories[ FileInstance     ::Directive() ] = new DerivedFactory< Shape , FileInstance >();
		ShapeList::ShapeFactories[ ShapeList        ::Directive() ] = new DerivedFactory< Shape , ShapeList >();
		ShapeList::ShapeFactories[ TriangleList     ::Directive() ] = new DerivedFactory< Shape , TriangleList >();
		ShapeList::ShapeFactories[ StaticAffineShape::Directive() ] = new DerivedFactory< Shape , StaticAffineShape >();
		ShapeList::ShapeFactories[ Union            ::Directive() ] = new DerivedFactory< Shape , Union >();
		ShapeList::ShapeFactories[ Intersection     ::Directive() ] = new DerivedFactory< Shape , Intersection >();
		ShapeList::ShapeFactories[ Difference       ::Directive() ] = new DerivedFactory< Shape , Difference >();

		GlobalSceneData::LightFactories[ DirectionalLight::Directive() ] = new DerivedFactory< Light , DirectionalLight >();
		GlobalSceneData::LightFactories[ PointLight      ::Directive() ] = new DerivedFactory< Light , PointLight >();
		GlobalSceneData::LightFactories[ SpotLight       ::Directive() ] = new DerivedFactory< Light , SpotLight >();

		ifstream istream;
		istream.open( InputRayFile.value );
		if( !istream ) THROW( "Failed to open file for reading: %s\n" , InputRayFile.value.c_str() );
		istream >> scene;
		Timer timer;
		Image32 img = scene.rayTrace( ImageWidth.value , ImageHeight.value , RecursionLimit.value , CutOffThreshold.value);
		std::cout << "Ray-traced " << ImageWidth.value << "x" << ImageHeight.value << " image in " << timer.elapsed() << " seconds" << std::endl;

		if( OutputImageFile.set ) img.write( OutputImageFile.value );
	}
	catch( const exception &e )
	{
		cerr << e.what() << endl;
		return EXIT_FAILURE;
	}

	for( auto iter=ShapeList::ShapeFactories.begin() ; iter!=ShapeList::ShapeFactories.end() ; iter++ ) delete iter->second;
	for( auto iter=GlobalSceneData::LightFactories.begin() ; iter!=GlobalSceneData::LightFactories.end() ; iter++ ) delete iter->second;

	return EXIT_SUCCESS;
}