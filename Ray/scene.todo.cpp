#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"

using namespace Ray;
using namespace Util;
using std::vector;

///////////
// Scene //
///////////
Point3D Scene::Reflect( Point3D v , Point3D n )
{
	return v - (2 * n * v.dot(n));
}

bool Scene::Refract( Point3D v , Point3D n , double ir , Point3D& refract )
{
	double cosIncident = -(v.dot(n));
	double sinIncident = sqrt(1 - cosIncident * cosIncident);
	if (fabs(ir * sinIncident) > 1)
	{
		refract = Point3D();
		return false;
	}
	else
	{
		double cosRefraction = sqrt(1 - ir * ir * (1 - cosIncident * cosIncident));
		refract = (v * ir + (n * (ir * cosIncident - cosRefraction))).normalize();
		return true;
	}
}

Point3D Scene::getColor( Ray3D ray , int rDepth , Point3D cLimit )
{
	RayShapeIntersectionInfo iInfo;
	double intersectionTime = this->intersect(ray, iInfo);
	if (intersectionTime == Infinity)
	{
		return Point3D();
	}
	else
	{
		Point3D result = Point3D();
		result = result + iInfo.material->emissive;

		for (vector<Light*>::const_iterator it = this->_globalData.lights.cbegin(); it != this->_globalData.lights.cend(); ++it)
		{
			result = result + (**it).getAmbient(ray, iInfo);

			Point3D transparency = (**it).transparency(iInfo, *this, Point3D(1e-5, 1e-5, 1e-5));
			result = result + ((**it).getDiffuse(ray, iInfo) + (**it).getSpecular(ray, iInfo)) * transparency;
		}

		if (iInfo.material->tex != nullptr)
		{
			const Image::Image32 &textureImage = iInfo.material->tex->_image;
			double textureHeight = textureImage.height();
			double textureWidth = textureImage.width();
			Point2D sampleLocation = iInfo.texture * Point2D(textureWidth, textureHeight);
			Image::Pixel32 sample = iInfo.material->tex->_image.bilinearSample(sampleLocation);
			Point3D textureColor = Point3D(sample.r / 255.0, sample.g / 255.0, sample.b / 255.0);
			result = result * textureColor;
		}

		if (ray.direction.dot(iInfo.normal) < 0)
		{
			Ray3D reflectionRay = Ray3D();
			reflectionRay.direction = this->Reflect(ray.direction, iInfo.normal);
			reflectionRay.position = iInfo.position + (reflectionRay.direction * Epsilon);
			if (rDepth > 0 && (iInfo.material->specular[0] > cLimit[0] || iInfo.material->specular[0] > cLimit[0] || iInfo.material->specular[0] > cLimit[0]))
				result = result + (this->getColor(reflectionRay, rDepth - 1, cLimit / iInfo.material->specular) * iInfo.material->specular);
		}

		Ray3D refractionRay = Ray3D();
		Point3D surfaceNormal = (ray.direction.dot(iInfo.normal) > 0) ? iInfo.normal * -1 : iInfo.normal;
		double refractiveIndex = (ray.direction.dot(iInfo.normal) > 0) ? iInfo.material->ir : 1 / iInfo.material->ir;
		bool canRefract = this->Refract(ray.direction, surfaceNormal, refractiveIndex, refractionRay.direction);
		if (canRefract)
		{
			refractionRay.position = iInfo.position + (refractionRay.direction * Epsilon);
			if (rDepth > 0 && (iInfo.material->transparent[0] > cLimit[0] || iInfo.material->transparent[1] > cLimit[1] || iInfo.material->transparent[2] > cLimit[2]))
				result = result + (this->getColor(refractionRay, rDepth - 1, cLimit / iInfo.material->transparent) * iInfo.material->transparent);
		}

		for (int i = 0; i <= 2; i++)
			result[i] = (result[i] > 1) ? 1 : result[i];
		return result;
	}
}

//////////////
// Material //
//////////////
void Material::drawOpenGL( GLSLProgram * glslProgram ) const
{
	float ambient[4] = { this->ambient[0], this->ambient[1], this->ambient[2], 1.0 };
	float diffuse[4] = { this->diffuse[0], this->diffuse[1], this->diffuse[2], 1.0 };
	float specular[4] = { this->specular[0], this->specular[1], this->specular[2], 1.0 };
	float emissive[4] = { this->emissive[0], this->emissive[1], this->emissive[2], 1.0 };
	float shininess = this->specularFallOff;

	glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	if (this->tex != nullptr)
	{
		glBindTexture(GL_TEXTURE_2D, this->tex->_openGLHandle);
		/*glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);*/
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glEnable(GL_TEXTURE_2D);
	}
	else
	{
		glDisable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

/////////////
// Texture //
/////////////
void Texture::initOpenGL( void )
{
	const Image::Image32& img = this->_image;
	size_t height = img.height();
	size_t width = img.width();
	unsigned char* pixels = new unsigned char[width * height * 4];
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int index = (i * width + j) * 4;
			pixels[index] = img(i, j).r;
			pixels[index + 1] = img(i, j).g;
			pixels[index + 2] = img(i, j).b;
			pixels[index + 3] = img(i, j).a;
		}
	}

	glGenTextures(1, &this->_openGLHandle);
	glBindTexture(GL_TEXTURE_2D, this->_openGLHandle);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid*)&pixels[0]);
	glBindTexture(GL_TEXTURE_2D, 0);

	delete[] pixels;

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();	
}

MeshTriangle::MeshTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
	this->mv0 = v0;
	this->mv1 = v1;
	this->mv2 = v2;
}