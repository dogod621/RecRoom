#pragma once

#include <pcl/point_types.h>

#include "Common.h"

namespace RecRoom
{
	//
	struct EIGEN_ALIGN16 _PointRAW
	{
		PCL_ADD_POINT4D;
		PCL_ADD_RGB;
		PCL_ADD_INTENSITY;
		float curvature;
		union { uint32_t serialNumber; int32_t hasSerialNumber; };

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointREC
	{
		PCL_ADD_POINT4D;
		PCL_ADD_RGB;
		PCL_ADD_NORMAL4D; 
		float curvature;
		float diffuseAlbedo;
		float specularAlbedo;
		float specularSharpness;
		union { uint32_t label; int32_t hasLabel; };

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointMED
	{
		PCL_ADD_POINT4D;
		PCL_ADD_RGB;
		PCL_ADD_INTENSITY;
		PCL_ADD_NORMAL4D;
		float curvature;
		float diffuseAlbedo;
		float specularAlbedo;
		float specularSharpness;
		union { uint32_t serialNumber; int32_t hasSerialNumber; };
		union { uint32_t label; int32_t hasLabel; };

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointVNN
	{
		PCL_ADD_POINT4D;
		uint32_t k;
		uint32_t* indices;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointNDF
	{
		PCL_ADD_POINT4D;
		PCL_ADD_NORMAL4D; 
		PCL_ADD_INTENSITY;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointLF
	{
		PCL_ADD_POINT4D;
		PCL_ADD_NORMAL4D; // Infact this is light dir
		PCL_ADD_INTENSITY;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	//
	struct PointMED;
	struct PointRAW;
	struct PointREC;

	struct PointRAW : public _PointRAW
	{
		inline PointRAW()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			r = g = b = 0; a = 255;
			intensity = 0.f;
			hasSerialNumber = -1;
		}

		inline PointRAW(const PointRAW& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			rgba = p.rgba;
			intensity = p.intensity;
			serialNumber = p.serialNumber;
		}

		inline PointRAW(const PointREC& p);
		inline PointRAW(const PointMED& p);

		inline PointRAW& operator = (const PointREC &p);
		inline PointRAW& operator = (const PointMED &p);

		inline bool HasSerialNumber() const { return (hasSerialNumber != -1); }
	};

	struct PointREC : public _PointREC
	{
		inline PointREC()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			r = g = b = 0; a = 255;
			normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
			diffuseAlbedo = 0.0f;
			specularAlbedo = 0.f;
			specularSharpness = 0.0f;
			hasLabel = -1;
		}

		inline PointREC(const PointREC& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			rgba = p.rgba;
			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
			diffuseAlbedo = p.diffuseAlbedo;
			specularAlbedo = p.specularAlbedo;
			specularSharpness = p.specularSharpness;
			label = p.label;
		}

		inline PointREC(const PointRAW& p);
		inline PointREC(const PointMED& p);

		inline PointREC& operator = (const PointRAW &p);
		inline PointREC& operator = (const PointMED &p);

		inline bool HasLabel() const { return (hasLabel != -1); }
	};

	struct PointMED : public _PointMED
	{
		inline PointMED()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			r = g = b = 0; a = 255;
			intensity = 0.f;
			normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
			diffuseAlbedo = 0.0f;
			specularAlbedo = 0.f;
			specularSharpness = 0.0f;
			hasSerialNumber = -1;
			hasLabel = -1;
		}

		inline PointMED(const PointMED& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			rgba = p.rgba;
			intensity = p.intensity;
			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
			diffuseAlbedo = p.diffuseAlbedo;
			specularAlbedo = p.specularAlbedo;
			specularSharpness = p.specularSharpness;
			serialNumber = p.serialNumber;
			label = p.label;
		}

		inline PointMED(const PointRAW& p);
		inline PointMED(const PointREC& p);

		inline PointMED& operator = (const PointRAW &p);
		inline PointMED& operator = (const PointREC &p);

		inline bool HasSerialNumber() const { return (hasSerialNumber != -1); }
		inline bool HasLabel() const { return (hasLabel != -1); }
	};

	struct PointVNN : public _PointVNN
	{
		inline PointVNN()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			k = 0;
			indices = nullptr;
		}

		inline PointVNN(const PointVNN& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			k = p.k;
			indices = p.indices;
		}

		template<class PointType>
		inline PointVNN(const PointType& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			k = 0;
			indices = nullptr;
		}
	};

	struct PointNDF : public _PointNDF
	{
		inline PointNDF()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			normal_x = normal_y = normal_z = data_n[3] = 0.f;
			intensity = 0.f;
		}

		inline PointNDF(const PointNDF& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f;
			intensity = p.intensity;
		}

		inline PointNDF(float normalX, float normalY, float normalZ, uint32_t label_, uint32_t serialNumber_, float intensity_)
		{
			x = ((float)label_) + 0.5f; 
			y = ((float)serialNumber_) + 0.5f;
			z = 0.5f; data[3] = 1.0f;
			normal_x = normalX; normal_y = normalY; normal_z = normalZ; data_n[3] = 0.f;
			intensity = intensity_;
		}
	};

	struct PointLF : public _PointLF
	{
		inline PointLF()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			normal_x = normal_y = normal_z = data_n[3] = 0.f;
			intensity = 0.f;
		}

		inline PointLF(const PointLF& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f;
			intensity = p.intensity;
		}
	};

	using PcRAW = Pc<PointRAW>;
	using PcMED = Pc<PointMED>;
	using PcREC = Pc<PointREC>;
	using PcVNN = Pc<PointVNN>;
	using PcNDF = Pc<PointNDF>;
	using PcLF = Pc<PointLF>;

	using AccRAW = Acc<PointRAW>;
	using AccMED = Acc<PointMED>;
	using AccREC = Acc<PointREC>;
	using AccVNN = Acc<PointVNN>;
	using AccNDF = Acc<PointNDF>;
	using AccLF = Acc<PointLF>;

	using KDTreeRAW = KDTree<PointRAW>;
	using KDTreeMED = KDTree<PointMED>;
	using KDTreeREC = KDTree<PointREC>;
	using KDTreeVNN = KDTree<PointVNN>;
	using KDTreeNDF = KDTree<PointNDF>;
	using KDTreeLF = KDTree<PointLF>;
}

//
POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointRAW,
(float, x, x) (float, y, y) (float, z, z)
(uint32_t, rgba, rgba)
(float, intensity, intensity)
(uint32_t, serialNumber, serialNumber)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointRAW, RecRoom::PointRAW)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointREC,
(float, x, x) (float, y, y) (float, z, z)
(uint32_t, rgba, rgba)
(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
(float, diffuseAlbedo, diffuseAlbedo) (float, specularAlbedo, specularAlbedo) (float, specularSharpness, specularSharpness)
(uint32_t, label, label)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointREC, RecRoom::PointREC)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointMED,
(float, x, x) (float, y, y) (float, z, z)
(uint32_t, rgba, rgba)
(float, intensity, intensity)
(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
(float, diffuseAlbedo, diffuseAlbedo) (float, specularAlbedo, specularAlbedo) (float, specularSharpness, specularSharpness)
(uint32_t, serialNumber, serialNumber)
(uint32_t, label, label)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointMED, RecRoom::PointMED)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointVNN,
(float, x, x) (float, y, y) (float, z, z)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointVNN, RecRoom::PointVNN)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointNDF,
(float, x, x) (float, y, y) (float, z, z)
(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z)
(float, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointNDF, RecRoom::PointNDF)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointLF,
(float, x, x) (float, y, y) (float, z, z)
(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z)
(float, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointLF, RecRoom::PointLF)

#include "Point.hpp"