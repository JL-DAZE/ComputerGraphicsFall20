#include <algorithm>
#include "image.h"
#include <stdlib.h>
#include <math.h>
#include <Util/exceptions.h>

using namespace Util;
using namespace Image;

unsigned char Image32::clamp(double n)
{
	unsigned char result;
	if (n < 0)
	{
		result = 0;
	}
	else if (n > 255)
	{
		result = 255;
	}
	else
	{
		result = static_cast<unsigned char>(n);
	}

	return result;
}

unsigned char Image32::noisifyChannel(unsigned char channelValue, double noiseFraction) const
{
	double noise = (static_cast<double>(rand()) / RAND_MAX * 2 - 1) * 255 * noiseFraction;
	unsigned char result = clamp(noise + channelValue);
	return result;
}

double Image32::computePixelLuminance(const Pixel32& p) const
{
	return 0.30 * p.r + 0.59 * p.g + 0.11 * p.b;
}

unsigned char Image32::quantizeChannel(double channelValue, double interval) const
{
	double temp = channelValue / interval;
	temp = round(temp) * interval;
	return clamp(temp);
}

int Image32::findMatrixEntry(int i, int j) const
{
	i = i % 2;
	j = j % 2;
	if (i == 0 && j == 0)
		return 1;
	else if (i == 0 && j == 1)
		return 3;
	else if (i == 1 && j == 0)
		return 4;
	else
		return 2;
}

unsigned char Image32::quantizeChannel(double channelValue, double interval, double threshold) const
{
	double temp = channelValue / interval;
	double decimal = temp - floor(temp);
	if (decimal > threshold)
		temp = ceil(temp) * interval;
	else
		temp = floor(temp) * interval;

	return clamp(temp);
}

void Image32::disperseError(double error, int i, int j, std::vector<double>& channel) const
{
	int index = i * this->_width + j;
	int width = this->_width;
	if (isValidPixelPosition(i, j + 1))
		channel[static_cast<long long>(index) + 1] += error * (static_cast<double>(7) / 16);
	if (isValidPixelPosition(i + 1, j - 1))
		channel[static_cast<long long>(index) + width - 1] += error * (static_cast<double>(3) / 16);
	if (isValidPixelPosition(i + 1, j))
		channel[static_cast<long long>(index) + width] += error * (static_cast<double>(5) / 16);
	if (isValidPixelPosition(i + 1, j + 1))
		channel[static_cast<long long>(index) + width + 1] += error * (static_cast<double>(1) / 16);
}

bool Image32::isValidPixelPosition(int x, int y) const
{
	return (x >= 0 && x < this->_height && y >= 0 && y < this->_width);
}

bool Image32::isValidPixelIndex(int index) const
{
	return (index >= 0 && index <= this->_width * this->_height);
}

Pixel32 Image32::blurPixel(int x, int y) const
{
	double sumR = 0;
	double sumG = 0;
	double sumB = 0;

	for (int i = x - 1; i <= x + 1; i++)
	{
		for (int j = y - 1; j <= y + 1; j++)
		{
			int index = i * this->_width + j;
			if (isValidPixelPosition(i, j))
			{
				int temp = abs(i - x) + abs(j - y);
				switch (temp)
				{
				case 0:
					sumR += this->_pixels[index].r * static_cast<double>(4) / 16;
					sumG += this->_pixels[index].g * static_cast<double>(4) / 16;
					sumB += this->_pixels[index].b * static_cast<double>(4) / 16;
					break;

				case 1:
					sumR += this->_pixels[index].r * static_cast<double>(2) / 16;
					sumG += this->_pixels[index].g * static_cast<double>(2) / 16;
					sumB += this->_pixels[index].b * static_cast<double>(2) / 16;
					break;

				case 2:
					sumR += this->_pixels[index].r * static_cast<double>(1) / 16;
					sumG += this->_pixels[index].g * static_cast<double>(1) / 16;
					sumB += this->_pixels[index].b * static_cast<double>(1) / 16;
					break;

				default:
					break;
				}
			}
		}
	}

	Pixel32 resultPixel = Pixel32();
	resultPixel.r = clamp(round(sumR));
	resultPixel.g = clamp(round(sumG));
	resultPixel.b = clamp(round(sumB));
	resultPixel.a = this->_pixels[x * this->_width + y].a;

	return resultPixel;
}

std::tuple<double, double, double> Image32::edgeDetectFilter(int x, int y) const
{
	double valueR = 0;
	double valueG = 0;
	double valueB = 0;

	for (int i = x - 1; i <= x + 1; i++)
	{
		for (int j = y - 1; j <= y + 1; j++)
		{
			int index = i * this->_width + j;
			if (isValidPixelPosition(i, j))
			{
				int temp = abs(i - x) + abs(j - y);
				switch (temp)
				{
				case 0:
					valueR += this->_pixels[index].r * static_cast<double>(1);
					valueG += this->_pixels[index].g * static_cast<double>(1);
					valueB += this->_pixels[index].b * static_cast<double>(1);
					break;

				case 1:
				case 2:
					valueR += this->_pixels[index].r * static_cast<double>(-1) / 8;
					valueG += this->_pixels[index].g * static_cast<double>(-1) / 8;
					valueB += this->_pixels[index].b * static_cast<double>(-1) / 8;
					break;

				default:
					break;
				}
			}
		}
	}

	return std::make_tuple(valueR, valueG, valueB);
}

std::tuple<double, double> Image32::computeMeanAndStandardDeviation(std::vector<double>& v) const
{
	if (v.empty())
		return std::make_tuple(0.0, 0.0);

	double mean = 0;
	double sd = 0;

	for (std::vector<double>::const_iterator it = v.cbegin(); it != v.cend(); ++it)
		mean += *it;
	mean = mean / v.size();

	for (std::vector<double>::const_iterator it = v.cbegin(); it != v.cend(); ++it)
		sd += pow(*it - mean, 2);
	sd = sqrt(sd / v.size());

	return std::make_tuple(mean, sd);
}

bool Image32::isWithinRadius(int x, int y, Point2D p, double radius) const
{
	double dist = sqrt(pow(x - p[0], 2) + pow(y - p[1], 2));
	return (dist <= radius);
}

double Image32::calculateGaussian(int x, int y, Point2D p, double variance) const
{
	double weightX = exp(-pow(x - p[0], 2) / (2 * variance));
	double weightY = exp(-pow(y - p[1], 2) / (2 * variance));
	return (weightX * weightY);
}

double Image32::max(double x, double y)
{
	if (x >= y)
		return x;
	else
		return y;
}

double Image32::max(double x, double y, double z, double w)
{
	double largest = x;
	if (y > largest)
		largest = y;
	if (z > largest)
		largest = z;
	if (w > largest)
		largest = w;
	return largest;
}

double Image32::min(double x, double y, double z, double w)
{
	double smallest = x;
	if (y < smallest)
		smallest = y;
	if (z < smallest)
		smallest = z;
	if (w < smallest)
		smallest = w;
	return smallest;
}

void Image32::calculateSizeAfterRotation(double angle, std::vector<double>& size) const
{
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double angleRadian = angle * Pi / 180;

	double topLeftPosX = 0;
	double topLeftPosY = 0;
	double topRightPosX = -srcWidth * sin(angleRadian);
	double topRightPosY = srcWidth * cos(angleRadian);
	double bottomLeftPosX = srcHeight * cos(angleRadian);
	double bottomLeftPosY = srcHeight * sin(angleRadian);
	double bottomRightPosX = srcHeight * cos(angleRadian) - srcWidth * sin(angleRadian);
	double bottomRightPosY = srcHeight * sin(angleRadian) + srcWidth * cos(angleRadian);

	double resultTopLeftX = min(topLeftPosX, topRightPosX, bottomLeftPosX, bottomRightPosX);
	double resultTopLeftY = min(topLeftPosY, topRightPosY, bottomLeftPosY, bottomRightPosY);
	double resultBottomRightX = max(topLeftPosX, topRightPosX, bottomLeftPosX, bottomRightPosX);
	double resultBottomRightY = max(topLeftPosY, topRightPosY, bottomLeftPosY, bottomRightPosY);

	size.push_back(resultBottomRightX - resultTopLeftX);
	size.push_back(resultBottomRightY - resultTopLeftY);
	size.push_back(resultTopLeftX);
	size.push_back(resultTopLeftY);
}

/////////////
// Image32 //
/////////////
Image32 Image32::addRandomNoise( double noise ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	for (int i = 0; i < numPixels; i++)
	{
		resultImage._pixels[i].r = noisifyChannel(this->_pixels[i].r, noise);
		resultImage._pixels[i].g = noisifyChannel(this->_pixels[i].g, noise);
		resultImage._pixels[i].b = noisifyChannel(this->_pixels[i].b, noise);
	}

	return resultImage;
}

Image32 Image32::brighten( double brightness ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	for (int i = 0; i < numPixels; i++)
	{
		int tempR = static_cast<int>(this->_pixels[i].r * brightness);
		int tempG = static_cast<int>(this->_pixels[i].g * brightness);
		int tempB = static_cast<int>(this->_pixels[i].b * brightness);
		resultImage._pixels[i].r = clamp(tempR);
		resultImage._pixels[i].g = clamp(tempG);
		resultImage._pixels[i].b = clamp(tempB);
	}

	return resultImage;
}

Image32 Image32::luminance( void ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	for (int i = 0; i < numPixels; i++)
	{
		unsigned char pixelLuminance = static_cast<unsigned char>(computePixelLuminance(this->_pixels[i]));
		resultImage._pixels[i].r = pixelLuminance;
		resultImage._pixels[i].g = pixelLuminance;
		resultImage._pixels[i].b = pixelLuminance;
	}

	return resultImage;
}

Image32 Image32::contrast( double contrast ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	double meanLuminance = 0;
	for (int i = 0; i < numPixels; i++)
	{
		double pixelLuminance = computePixelLuminance(this->_pixels[i]);
		meanLuminance += pixelLuminance;
	}
	meanLuminance = meanLuminance / numPixels;

	for (int i = 0; i < numPixels; i++)
	{
		double tempR = meanLuminance + (this->_pixels[i].r - meanLuminance) * contrast;
		double tempG = meanLuminance + (this->_pixels[i].g - meanLuminance) * contrast;
		double tempB = meanLuminance + (this->_pixels[i].b - meanLuminance) * contrast;
		resultImage._pixels[i].r = clamp(tempR);
		resultImage._pixels[i].g = clamp(tempG);
		resultImage._pixels[i].b = clamp(tempB);
	}

	return resultImage;
}

Image32 Image32::saturate( double saturation ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	for (int i = 0; i < numPixels; i++)
	{
		double pixelLuminance = computePixelLuminance(this->_pixels[i]);
		double tempR = pixelLuminance + (this->_pixels[i].r - pixelLuminance) * saturation;
		double tempG = pixelLuminance + (this->_pixels[i].g - pixelLuminance) * saturation;
		double tempB = pixelLuminance + (this->_pixels[i].b - pixelLuminance) * saturation;
		resultImage._pixels[i].r = clamp(tempR);
		resultImage._pixels[i].g = clamp(tempG);
		resultImage._pixels[i].b = clamp(tempB);
	}

	return resultImage;
}

Image32 Image32::quantize( int bits ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	if (bits > 8 || bits <= 0)
	{
		std::cout << "Bad input." << std::endl;
		return Image32();
	}

	double interval = 255 / (pow(2, bits) - 1);
	for (int i = 0; i < numPixels; i++)
	{
		resultImage._pixels[i].r = quantizeChannel(this->_pixels[i].r, interval);
		resultImage._pixels[i].g = quantizeChannel(this->_pixels[i].g, interval);
		resultImage._pixels[i].b = quantizeChannel(this->_pixels[i].b, interval);
		resultImage._pixels[i].a = quantizeChannel(this->_pixels[i].a, interval);
	}

	return resultImage;
}

Image32 Image32::randomDither( int bits ) const
{
	Image32 resultImage = Image32(*this);
	int numPixels = this->_width * this->_height;
	if (bits > 8 || bits <= 0)
	{
		std::cout << "Bad input." << std::endl;
		return Image32();
	}

	double interval = 255 / (pow(2, bits) - 1);
	double noiseFraction = 1 / pow(2, bits);
	for (int i = 0; i < numPixels; i++)
	{
		resultImage._pixels[i].r = noisifyChannel(resultImage._pixels[i].r, noiseFraction);
		resultImage._pixels[i].r = quantizeChannel(resultImage._pixels[i].r, interval);
		resultImage._pixels[i].g = noisifyChannel(resultImage._pixels[i].g, noiseFraction);
		resultImage._pixels[i].g = quantizeChannel(resultImage._pixels[i].g, interval);
		resultImage._pixels[i].b = noisifyChannel(resultImage._pixels[i].b, noiseFraction);
		resultImage._pixels[i].b = quantizeChannel(resultImage._pixels[i].b, interval);
		resultImage._pixels[i].a = quantizeChannel(resultImage._pixels[i].a, interval);
	}

	return resultImage;
}

Image32 Image32::orderedDither2X2( int bits ) const
{
	Image32 resultImage = Image32(*this);
	if (bits > 8 || bits <= 0)
	{
		std::cout << "Bad input." << std::endl;
		return Image32();
	}

	int height = this->_height;
	int width = this->_width;
	double interval = 255 / (pow(2, bits) - 1);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			int matrixEntry = findMatrixEntry(i, j);
			double threshold = static_cast<double>(matrixEntry) / 5;
			resultImage._pixels[index].r = quantizeChannel(resultImage._pixels[index].r, interval, threshold);
			resultImage._pixels[index].g = quantizeChannel(resultImage._pixels[index].g, interval, threshold);
			resultImage._pixels[index].b = quantizeChannel(resultImage._pixels[index].b, interval, threshold);
			resultImage._pixels[index].a = quantizeChannel(resultImage._pixels[index].a, interval);
		}
	}

	return resultImage;
}

Image32 Image32::floydSteinbergDither( int bits ) const
{
	Image32 resultImage = Image32(*this);
	if (bits > 8 || bits <= 0)
	{
		std::cout << "Bad input." << std::endl;
		return Image32();
	}

	int height = this->_height;
	int width = this->_width;
	double interval = 255 / (pow(2, bits) - 1);

	// Copy the channel values with double type for error diffusion.
	std::vector<double> rChannel;
	std::vector<double> gChannel;
	std::vector<double> bChannel;
	for (int i = 0; i < height * width; i++)
	{
		rChannel.push_back(this->_pixels[i].r);
		gChannel.push_back(this->_pixels[i].g);
		bChannel.push_back(this->_pixels[i].b);
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			resultImage._pixels[index].r = quantizeChannel(rChannel[index], interval);
			resultImage._pixels[index].g = quantizeChannel(gChannel[index], interval);
			resultImage._pixels[index].b = quantizeChannel(bChannel[index], interval);
			resultImage._pixels[index].a = quantizeChannel(this->_pixels[index].a, interval);

			double errorR = this->_pixels[index].r - resultImage._pixels[index].r;
			double errorG = this->_pixels[index].g - resultImage._pixels[index].g;
			double errorB = this->_pixels[index].b - resultImage._pixels[index].b;
			disperseError(errorR, i, j, rChannel);
			disperseError(errorG, i, j, gChannel);
			disperseError(errorB, i, j, bChannel);
		}
	}

	return resultImage;
}

Image32 Image32::blur3X3( void ) const
{
	Image32 resultImage = Image32(*this);
	int height = this->_height;
	int width = this->_width;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			resultImage._pixels[index] = blurPixel(i, j);
		}
	}

	return resultImage;
}

Image32 Image32::edgeDetect3X3( void ) const
{
	Image32 resultImage = Image32(*this);
	int height = this->_height;
	int width = this->_width;

	// Vectors for storing the edge values.
	std::vector<double> edgeValueR;
	std::vector<double> edgeValueG;
	std::vector<double> edgeValueB;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			std::tuple<double, double, double> filteredPixel = edgeDetectFilter(i, j);
			edgeValueR.push_back(std::get<0>(filteredPixel));
			edgeValueG.push_back(std::get<1>(filteredPixel));
			edgeValueB.push_back(std::get<2>(filteredPixel));
		}
	}

	// Compute the mean and standard deviation.
	std::tuple<double, double> computeResultR = computeMeanAndStandardDeviation(edgeValueR);
	std::tuple<double, double> computeResultG = computeMeanAndStandardDeviation(edgeValueG);
	std::tuple<double, double> computeResultB = computeMeanAndStandardDeviation(edgeValueB);
	double meanR = std::get<0>(computeResultR);
	double sdR = std::get<1>(computeResultR);
	double meanG = std::get<0>(computeResultG);
	double sdG = std::get<1>(computeResultG);
	double meanB = std::get<0>(computeResultB);
	double sdB = std::get<1>(computeResultB);


	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			double zScoreR = fabs((edgeValueR[index] - meanR) / sdR);
			double zScoreG = fabs((edgeValueG[index] - meanG) / sdG);
			double zScoreB = fabs((edgeValueB[index] - meanB) / sdB);
			resultImage._pixels[index].r = clamp(85 * zScoreR);
			resultImage._pixels[index].g = clamp(85 * zScoreG);
			resultImage._pixels[index].b = clamp(85 * zScoreB);
		}
	}

	return resultImage;
}

Image32 Image32::scaleNearest( double scaleFactor ) const
{
	Image32 resultImage = Image32();
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double destWidthD = scaleFactor * srcWidth;
	double destHeightD = scaleFactor * srcHeight;
	int destWidthI = static_cast<int>(floor(destWidthD));
	int destHeightI = static_cast<int>(floor(destHeightD));
	resultImage.setSize(destWidthI, destHeightI);

	for (int i = 0; i < destHeightI; i++)
	{
		for (int j = 0; j < destWidthI; j++)
		{
			int index = i * destWidthI + j;
			double srcX = i / destHeightD * srcHeight;
			double srcY = j / destWidthD * srcWidth;
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = nearestSample(p);
		}
	}

	return resultImage;
}

Image32 Image32::scaleBilinear( double scaleFactor ) const
{
	Image32 resultImage = Image32();
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double destWidthD = scaleFactor * srcWidth;
	double destHeightD = scaleFactor * srcHeight;
	int destWidthI = static_cast<int>(floor(destWidthD));
	int destHeightI = static_cast<int>(floor(destHeightD));
	resultImage.setSize(destWidthI, destHeightI);

	for (int i = 0; i < destHeightI; i++)
	{
		for (int j = 0; j < destWidthI; j++)
		{
			int index = i * destWidthI + j;
			double srcX = i / destHeightD * srcHeight;
			double srcY = j / destWidthD * srcWidth;
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = bilinearSample(p);
		}
	}

	return resultImage;
}

Image32 Image32::scaleGaussian( double scaleFactor ) const
{
	Image32 resultImage = Image32();
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double destWidthD = scaleFactor * srcWidth;
	double destHeightD = scaleFactor * srcHeight;
	int destWidthI = static_cast<int>(floor(destWidthD));
	int destHeightI = static_cast<int>(floor(destHeightD));
	resultImage.setSize(destWidthI, destHeightI);

	double radius = 1 / scaleFactor;
	double variance = pow(max(static_cast<double>(1), 1 / scaleFactor) * 0.75, 2);

	for (int i = 0; i < destHeightI; i++)
	{
		for (int j = 0; j < destWidthI; j++)
		{
			int index = i * destWidthI + j;
			double srcX = i / destHeightD * srcHeight;
			double srcY = j / destWidthD * srcWidth;
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = gaussianSample(p, variance, radius);
		}
	}

	return resultImage;
}

Image32 Image32::rotateNearest( double angle ) const
{
	Image32 resultImage = Image32();
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double angleRadian = angle * Pi / 180;

	std::vector<double> resultImageSizeAndOffset;
	calculateSizeAfterRotation(angle, resultImageSizeAndOffset);
	double destHeightD = resultImageSizeAndOffset[0];
	double destWidthD = resultImageSizeAndOffset[1];
	int destHeightI = static_cast<int>(destHeightD);
	int destWidthI = static_cast<int>(destWidthD);
	double pixelOffsetX = resultImageSizeAndOffset[2];
	double pixelOffsetY = resultImageSizeAndOffset[3];
	resultImage.setSize(destWidthI, destHeightI);

	for (int i = 0; i < destHeightI; i++)
	{
		for (int j = 0; j < destWidthI; j++)
		{
			int index = i * destWidthI + j;
			double srcX = (i + pixelOffsetX) * cos(-angleRadian) - (j + pixelOffsetY) * sin(-angleRadian);
			double srcY = (i + pixelOffsetX) * sin(-angleRadian) + (j + pixelOffsetY) * cos(-angleRadian);
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = nearestSample(p);
		}
	}

	return resultImage;
}

Image32 Image32::rotateBilinear( double angle ) const
{
	Image32 resultImage = Image32();
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double angleRadian = angle * Pi / 180;

	std::vector<double> resultImageSizeAndOffset;
	calculateSizeAfterRotation(angle, resultImageSizeAndOffset);
	double destHeightD = resultImageSizeAndOffset[0];
	double destWidthD = resultImageSizeAndOffset[1];
	int destHeightI = static_cast<int>(destHeightD);
	int destWidthI = static_cast<int>(destWidthD);
	double pixelOffsetX = resultImageSizeAndOffset[2];
	double pixelOffsetY = resultImageSizeAndOffset[3];
	resultImage.setSize(destWidthI, destHeightI);

	for (int i = 0; i < destHeightI; i++)
	{
		for (int j = 0; j < destWidthI; j++)
		{
			int index = i * destWidthI + j;
			double srcX = (i + pixelOffsetX) * cos(-angleRadian) - (j + pixelOffsetY) * sin(-angleRadian);
			double srcY = (i + pixelOffsetX) * sin(-angleRadian) + (j + pixelOffsetY) * cos(-angleRadian);
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = bilinearSample(p);
		}
	}

	return resultImage;
}

Image32 Image32::rotateGaussian( double angle ) const
{
	Image32 resultImage = Image32();
	int srcWidth = this->_width;
	int srcHeight = this->_height;
	double angleRadian = angle * Pi / 180;

	std::vector<double> resultImageSizeAndOffset;
	calculateSizeAfterRotation(angle, resultImageSizeAndOffset);
	double destHeightD = resultImageSizeAndOffset[0];
	double destWidthD = resultImageSizeAndOffset[1];
	int destHeightI = static_cast<int>(destHeightD);
	int destWidthI = static_cast<int>(destWidthD);
	double pixelOffsetX = resultImageSizeAndOffset[2];
	double pixelOffsetY = resultImageSizeAndOffset[3];
	resultImage.setSize(destWidthI, destHeightI);

	double radius = 1;
	double variance = pow(0.75, 2);

	for (int i = 0; i < destHeightI; i++)
	{
		for (int j = 0; j < destWidthI; j++)
		{
			int index = i * destWidthI + j;
			double srcX = (i + pixelOffsetX) * cos(-angleRadian) - (j + pixelOffsetY) * sin(-angleRadian);
			double srcY = (i + pixelOffsetX) * sin(-angleRadian) + (j + pixelOffsetY) * cos(-angleRadian);
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = gaussianSample(p, variance, radius);
		}
	}

	return resultImage;
}

void Image32::setAlpha( const Image32& matte )
{
	if (matte._height != this->_height || matte._width != this->_width)
		return;

	for (int i = 0; i < this->_height * this->_width; i++)
	{
		this->_pixels[i].a = clamp((static_cast<double>(matte._pixels[i].r) + matte._pixels[i].g + matte._pixels[i].b) / 3);
	}
}

Image32 Image32::composite( const Image32& overlay ) const
{
	if (this->_height != overlay._height || this->_width != overlay._width)
		return Image32();

	Image32 resultImage = Image32(*this);
	int numPixels = this->_height * this->_width;
	for (int i = 0; i < numPixels; i++)
	{
		double alphaForeground = static_cast<double>(overlay._pixels[i].a) / 255;
		double alphaBackground = static_cast<double>(this->_pixels[i].a) / 255;
		double rChannel = overlay._pixels[i].r * alphaForeground;
		rChannel = rChannel + this->_pixels[i].r * (1 - alphaForeground) * alphaBackground;
		resultImage._pixels[i].r = clamp(rChannel);
		double gChannel = overlay._pixels[i].g * alphaForeground;
		gChannel = gChannel + this->_pixels[i].g * (1 - alphaForeground) * alphaBackground;
		resultImage._pixels[i].g = clamp(gChannel);
		double bChannel = overlay._pixels[i].b * alphaForeground;
		bChannel = bChannel + this->_pixels[i].b * (1 - alphaForeground) * alphaBackground;
		resultImage._pixels[i].b = clamp(bChannel);
	}

	return resultImage;
}

Image32 Image32::CrossDissolve( const Image32& source , const Image32& destination , double blendWeight )
{
	Image32 resultImage = Image32(source);
	int numPixels = source._height * source._width;

	for (int i = 0; i < numPixels; i++)
	{
		resultImage._pixels[i].r = clamp(source._pixels[i].r * (1 - blendWeight) + destination._pixels[i].r * blendWeight);
		resultImage._pixels[i].g = clamp(source._pixels[i].g * (1 - blendWeight) + destination._pixels[i].g * blendWeight);
		resultImage._pixels[i].b = clamp(source._pixels[i].b * (1 - blendWeight) + destination._pixels[i].b * blendWeight);
	}

	return resultImage;
}

Image32 Image32::warp( const OrientedLineSegmentPairs& olsp ) const
{
	Image32 resultImage = Image32(*this);
	int height = this->_height;
	int width = this->_width;

	double radius = 1;
	double variance = pow(0.75, 2);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			Point2D src = olsp.getSourcePosition(Point2D(static_cast<double>(i), static_cast<double>(j)));
			resultImage._pixels[index] = gaussianSample(src, variance, radius);
		}
	}

	return resultImage;
}

Image32 Image32::funFilter( void ) const
{
	Image32 resultImage = Image32(*this);
	int height = this->_height;
	int width = this->_width;

	double radius = 1;
	double variance = 0.5;

	double middleX = static_cast<double>(height) / 2;
	double middleY = static_cast<double>(width) / 2;
	double halfDiagonal = Point2D(middleX, middleY).length();

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = i * width + j;
			double diffX = i - middleX;
			double diffY = j - middleY;
			double dist = Point2D(diffX, diffY).length();
			// double srcX = middleX + pow(fabs(diffX / middleX), 1.5) * middleX * (diffX / fabs(diffX));
			// double srcY = middleY + pow(fabs(diffY / middleY), 1.5) * middleY * (diffY / fabs(diffY));
			double srcX = middleX + 1.5 * (dist / halfDiagonal) * diffX;
			double srcY = middleY + 1.5 * (dist / halfDiagonal) * diffY;
			Point2D p = Point2D(srcX, srcY);
			resultImage._pixels[index] = gaussianSample(p, variance, radius);
		}
	}

	return resultImage;
}

Image32 Image32::crop( int x1 , int y1 , int x2 , int y2 ) const
{
	Image32 resultImage = Image32();

	if (!(isValidPixelPosition(x1, y1) && isValidPixelPosition(x2, y2)))
	{
		std::cout << "Bad input." << std::endl;
		return Image32();
	}

	resultImage.setSize(y2 - y1 + 1, x2 - x1 + 1);
	for (int i = 0; i < resultImage._height; i++)
	{
		for (int j = 0; j < resultImage._width; j++)
		{
			resultImage._pixels[i * resultImage._width + j] = this->_pixels[(x1 + i) * this->_width + y1 + j];
		}
	}

	return resultImage;
}

Pixel32 Image32::nearestSample( Point2D p ) const
{
	int x = static_cast<int>(round(p[0]));
	int y = static_cast<int>(round(p[1]));
	int index = x * this->_width + y;

	if (isValidPixelPosition(x, y))
		return this->_pixels[index];		
	else
		return Pixel32();	
}

Pixel32 Image32::bilinearSample(Point2D p) const
{
	Pixel32 resultPixel = Pixel32();
	int x1 = static_cast<int>(floor(p[0]));
	int y1 = static_cast<int>(floor(p[1]));
	int x2 = x1 + 1;
	int y2 = y1 + 1;
	double dx = p[0] - x1;
	double dy = p[1] - y1;
	double aR = 0, aG = 0, aB = 0;
	double bR = 0, bG = 0, bB = 0;

	if (isValidPixelPosition(x1, y1))
	{
		int index = x1 * this->_width + y1;
		aR = aR + this->_pixels[index].r * (1 - dy);
		aG = aG + this->_pixels[index].g * (1 - dy);
		aB = aB + this->_pixels[index].b * (1 - dy);
	}
	if (isValidPixelPosition(x1, y2))
	{
		int index = x1 * this->_width + y2;
		aR = aR + this->_pixels[index].r * dy;
		aG = aG + this->_pixels[index].g * dy;
		aB = aB + this->_pixels[index].b * dy;
	}
	if (isValidPixelPosition(x2, y1))
	{
		int index = x2 * this->_width + y1;
		bR = bR + this->_pixels[index].r * (1 - dy);
		bG = bG + this->_pixels[index].g * (1 - dy);
		bB = bB + this->_pixels[index].b * (1 - dy);
	}
	if (isValidPixelPosition(x2, y2))
	{
		int index = x2 * this->_width + y2;
		bR = bR + this->_pixels[index].r * dy;
		bG = bG + this->_pixels[index].g * dy;
		bB = bB + this->_pixels[index].b * dy;
	}
	
	double resultR = aR * (1 - dx) + bR * dx;
	double resultG = aG * (1 - dx) + bG * dx;
	double resultB = aB * (1 - dx) + bB * dx;
	resultPixel.r = clamp(round(resultR));
	resultPixel.g = clamp(round(resultG));
	resultPixel.b = clamp(round(resultB));

	return resultPixel;
}
		
Pixel32 Image32::gaussianSample( Point2D p , double variance , double radius ) const
{
	Pixel32 resultPixel = Pixel32();
	int x1 = static_cast<int>(floor(p[0] - radius));
	int y1 = static_cast<int>(floor(p[1] - radius));
	int x2 = static_cast<int>(ceil(p[0] + radius));
	int y2 = static_cast<int>(ceil(p[1] + radius));

	double channelR = 0;
	double channelG = 0;
	double channelB = 0;
	double totalWeight = 0;

	for (int i = x1; i <= x2; i++)
	{
		for (int j = y1; j <= y2; j++)
		{
			int index = i * this->_width + j;
			if (isWithinRadius(i, j, p, radius))
			{
				double pixelWeight = calculateGaussian(i, j, p, variance);
				totalWeight += pixelWeight;
				if (isValidPixelPosition(i, j))
				{
					channelR += pixelWeight * this->_pixels[index].r;
					channelG += pixelWeight * this->_pixels[index].g;
					channelB += pixelWeight * this->_pixels[index].b;
				}
			}

		}
	}

	channelR = channelR / totalWeight;
	channelG = channelG / totalWeight;
	channelB = channelB / totalWeight;
	resultPixel.r = clamp(round(channelR));
	resultPixel.g = clamp(round(channelG));
	resultPixel.b = clamp(round(channelB));

	return resultPixel;
}
