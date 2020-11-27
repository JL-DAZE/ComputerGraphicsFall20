#ifndef IMAGE_INCLUDED
#define IMAGE_INCLUDED

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <Util/geometry.h>
#include "lineSegments.h"
#include <tuple>

namespace Image
{
	/** This class represents a 4-channel, 32-bit, RGBA pixel. */
	class Pixel32
	{
	public:
		/** The red component of the pixel */
		unsigned char r;

		/** The green component of the pixel */
		unsigned char g;

		/** The blue component of the pixel */
		unsigned char b;

		/** The alpha component of the pixel */
		unsigned char a;

		/** The default constructor instantiates a pixel with 0 for the red, green and blue components, and 255 for the alpha.*/
		Pixel32( void );
	};


	/** This class represents an RGBA image with 8 bits per channel. */
	class Image32
	{
		/** The dimensions of the image */
		int _width , _height;

		/** The pixel values */
		Pixel32* _pixels;

		/** The method validates that the pixel index is valid */
		void _assertInBounds( int x , int y ) const;
	public:
		/** The default constructor */
		Image32( void );

		/** The copy constructor copies pixel values */
		Image32( const Image32& img );

		/** The move constructor moves pixel ownership from the input to the new object. */
		Image32( Image32&& img );

		/** The copy assignment operator copies pixel values */
		Image32& operator = ( const Image32& img );

		/** The move assignment operator moves pixel ownership from the input to the new object. */
		Image32& operator = ( Image32&& img );

		/** The destructor deallocates memory associated with the image. */
		~Image32( void );

		/** This method sets the dimension of the image. */
		void setSize( int width , int height );

		/** This method returns the width of the image */
		int width( void ) const;

		/** This method returns the height of the image */
		int height( void ) const;

		/** This method returns a reference to the indexed pixel.
		*** An exception is thrown if the index is out of bounds. */
		Pixel32& operator() ( int x , int y );

		/** This method returns a reference to the indexed pixel.
		*** An exception is thrown if the index is out of bounds. */
		const Pixel32& operator() ( int x , int y ) const;

		/** This method reads in an image from the specified file. It uses the file extension to determine if the file should be read in as a BMP file or as a JPEG file. */
		void read( std::string fileName );

		/** This method writes in an image out to the specified file. It uses the file extension to determine if the file should be written out as a BMP file or as a JPEG file. */
		void write( std::string fileName ) const;

		/** This method outputs a new image image with random noise added to each pixel.
		*** The value of the input parameter should be in the range [0,1] representing the fraction
		*** of noise that should be added. The actual amount of noise added is in the range [-noise,noise]. */
		Image32 addRandomNoise( double noise ) const;

		/** This method outputs a new image in which each pixel is brightened.
		*** The value of the input parameter is the scale by which the image should be brightened. */
		Image32 brighten( double brightness ) const;

		/** This method outputs the gray-scale image. */
		Image32 luminance( void ) const;

		/** This method outputs a new image in which the contract has been changed.
		*** The value of the input parameter is the scale by which the contrast of the image should be changed. */
		Image32 contrast( double contrast ) const;

		/** This method outputs a new image in which the saturation of each pixel has been changed.
		*** The value of the input parameter is the scale by which the saturation of the pixel should be changed. */
		Image32 saturate( double saturation ) const;

		/** This method outputs a new image in which each pixel is represented by a fixed number of bits.
		*** The final pixel values are obtained by quantizing.
		*** The value of the input parameter is the number of bits that should be used to represent a color component in the output image. */
		Image32 quantize( int bits ) const;

		/** This method outputs a new image in which each pixel is represented by a fixed number of bits.
		*** The final pixel values are obtained by adding noise to the pixel color channels and then quantizing.
		*** The value of the input parameter is the number of bits that should be used to represent a color component in the output image. */
		Image32 randomDither( int bits ) const;

		/** This method outputs a new image in which each pixel is represented by a fixed number of bits.
		*** The final pixel values are obtained by using a 2x2 dithering matrix to determine how values should be quantized.
		*** The value of the input parameter is the number of bits that should be used to represent a color component in the output image. */
		Image32 orderedDither2X2( int bits ) const;

		/** This method outputs a new image in which each pixel is represented by a fixed number of bits.
		*** The final pixel values are obtained by using Floyd-Steinberg dithering for propogating quantization errors.
		*** The value of the input parameter is the number of bits that should be used to represent a color component in the output image. */
		Image32 floydSteinbergDither( int bits ) const;

		/** This method outputs a blur of the image using a 3x3 mask. */
		Image32 blur3X3( void ) const;

		/** This method outpus a new image highlighting the edges in the input using a 3x3 mask. */
		Image32 edgeDetect3X3( void ) const;

		/** This method outputs a scaled image which is obtained using nearest-point sampling.
		* The value of the input parameter is the factor by which the image is to be scaled.
		*/
		Image32 scaleNearest( double scaleFactor ) const;

		/** This method outputs a scaled image which is obtained using bilinear sampling.
		*** The value of the input parameter is the factor by which the image is to be scaled. */
		Image32 scaleBilinear( double scaleFactor ) const;

		/** This method outputs a scaled image which is obtained using Gaussian sampling.
		*** The value of the input parameter is the factor by which the image is to be scaled. */
		Image32 scaleGaussian( double scaleFactor ) const;

		/** This method outputs a rotated image which is obtained using nearest-point sampling.
		*** The value of the input parameter is the angle of rotation (in degrees). */
		Image32 rotateNearest( double angle ) const;

		/** This method outputs a rotated image which is obtained using bilinear sampling.
		*** The value of the input parameter is the angle of rotation (in degrees). */
		Image32 rotateBilinear( double angle ) const;

		/** This method outputs a rotated image which is obtained using Gaussian sampling.
		*** The value of the input parameter is the angle of rotation (in degrees). */
		Image32 rotateGaussian( double angle ) const;

		/** This method sets the alpha-channel of the current image using the information provided in the matte image.
		*** The method returns true if it has been implemented. */
		void setAlpha( const Image32& matte );

		/** This method outputs an image that is a composite of the current image and the overlay.
		*** The method uses the values in the alpha-channel of the overlay image to determine how pixels should be blended. */
		Image32 composite( const Image32& overlay ) const;

		/** This method outputs a croppedimage.
		*** The values of the input parameters specify the corners of the cropping rectangle. */
		Image32 crop( int x1 , int y1 , int x2 , int y2 ) const;

		/** This method outputs the results of a fun-filter. */
		Image32 funFilter( void ) const;

		/** This static method outputs the result a Beier-Neely morph.
		*** The method uses the set of line segment pairs to define correspondences between the source and destination image.
		*** The time-step parameter, in the range of [0,1], specifies the point in the morph at which the output image should be obtained. */
		static Image32 BeierNeelyMorph( const Image32& source , const Image32& destination , const OrientedLineSegmentPairs& olsp , double timeStep );

		/** This method outputs a warped image using the correspondences defined by the line segment pairs. */
		Image32 warp( const OrientedLineSegmentPairs& olsp ) const;

		/** This static method outputs the cross-dissolve of two image.
		*** The method generates an image which is the blend of the source and destination, using the blend-weight in the range [0,1] to
		*** determine what faction of the source and destination images should be used to generate the final output. */
		static Image32 CrossDissolve( const Image32& source , const Image32& destination , double blendWeight );

		/** This method returns the value of the image, sampled at position p using nearest-point sampling.
		*** The variance of the Gaussian and the radius over which the weighted summation is performed are specified by the parameters. */
		Pixel32 nearestSample( Util::Point2D p ) const;

		/** This method returns the value of the image, sampled at position p using bilinear-weighted sampling.
		*** The variance of the Gaussian and the radius over which the weighted summation is performed are specified by the parameters. */
		Pixel32 bilinearSample( Util::Point2D p ) const;

		/** This method returns the value of the image, sampled at position p using Gaussian-weighted sampling.
		*** The variance of the Gaussian and the radius over which the weighted summation is performed are specified by the parameters. */
		Pixel32 gaussianSample( Util::Point2D p , double variance , double radius ) const;


	/*Helper functions*/
	private:
		/** This method clamps an int value to the range of unsigned char. */
		static unsigned char clamp(double n);

		/** This method noisifies a channel with the fraction of noise specified. */
		unsigned char noisifyChannel(unsigned char channelValue, double noiseFraction) const;

		/** This method computes the luminance of a single pixel. */
		double computePixelLuminance(const Pixel32& p) const;

		/** This method quantize a channel. The value of interval should be pre-calculated as 255/(pow(2,bits)-1). */
		unsigned char quantizeChannel(double channelValue, double interval) const;

		/** This method is used by orderedDither2x2 to find a pixel's entry in the 2x2 matrix. */
		int findMatrixEntry(int i, int j) const;

		/** This method quantize a channel, but rounding is decided by threshold. Used in orderedDither2x2. */
		unsigned char quantizeChannel(double channelValue, double interval, double threshold) const;

		/** This method disperses the error to adjacent pixels. Used in floydSteinbergDither. */
		void disperseError(double error, int i, int j, std::vector<double>& channel) const;

		/** This method checks if a given x, y represents a valid pixel in image. */
		bool isValidPixelPosition(int x, int y) const;

		/** This method checks if a given index represents a valid pixel in image, a.k.a 0 <= index < numPixels */
		bool isValidPixelIndex(int index) const;

		/** This method blurs a single pixel. Used in blur3x3. */
		Pixel32 blurPixel(int x, int y) const;

		/** This method applies the edge detect filter on a pixel. */
		std::tuple<double, double, double> edgeDetectFilter(int x, int y) const;

		/** This method computes the mean value and the standard deviation of double values in vector. Used in edgeDetect3x3. */
		std::tuple<double, double> computeMeanAndStandardDeviation(std::vector<double>& v) const;

		/** This method checks if a given pixel is within a given radius of a given point. */
		bool isWithinRadius(int x, int y, Util::Point2D p, double radius) const;

		/** This method calculates the gaussian weight of a pixel. */
		double calculateGaussian(int x, int y, Util::Point2D p, double variance) const;

		/** This method returns the larger of two double values. */
		static double max(double x, double y);

		/** This method returns the largest among four double values. */
		static double max(double x, double y, double z, double w);

		/** This method returns the smallest among four double values. */
		static double min(double x, double y, double z, double w);

		/** This method calculates the size and the positional offset of rotated image. */
		void calculateSizeAfterRotation(double angle, std::vector<double>& size) const;

	};
}
#endif // IMAGE_INCLUDED

