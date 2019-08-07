#pragma once

#include <pcl/point_types.h>

#include "Common.h"

namespace RecRoom
{
	// Macros

	// RGB
#ifdef INPUT_PERPOINT_RGB
#	define RAW_ADD_RGB						PCL_ADD_RGB;
#	define RAW_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define RAW_ISFINITE_RGB
#	define RAW_INIT_RGB						r = g = b = 0; a = 255;
#	define RAW_COPY_RGB						rgba = p.rgba;
#else
#	define RAW_ADD_RGB
#	define RAW_REGISTER_RGB
#	define RAW_ISFINITE_RGB
#	define RAW_INIT_RGB
#	define RAW_COPY_RGB
#endif

#ifdef OUTPUT_PERPOINT_RGB
#	define REC_ADD_RGB						PCL_ADD_RGB;
#	define REC_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define REC_ISFINITE_RGB
#	define REC_INIT_RGB						r = g = b = 0; a = 255;
#	define REC_COPY_RGB						rgba = p.rgba;
#else
#	define REC_ADD_RGB
#	define REC_REGISTER_RGB
#	define REC_ISFINITE_RGB
#	define REC_INIT_RGB
#	define REC_COPY_RGB
#endif

#ifdef PERPOINT_RGB
#	define MED_ADD_RGB						PCL_ADD_RGB;
#	define MED_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define MED_ISFINITE_RGB
#	define MED_INIT_RGB						r = g = b = 0; a = 255;
#	define MED_COPY_RGB						rgba = p.rgba;
#else
#	define MED_ADD_RGB
#	define MED_REGISTER_RGB
#	define MED_ISFINITE_RGB
#	define MED_INIT_RGB
#	define MED_COPY_RGB
#endif

// Intensity
#ifdef INPUT_PERPOINT_INTENSITY
#	define RAW_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define RAW_REGISTER_INTENSITY			(float, intensity, intensity)
#	define RAW_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define RAW_INIT_INTENSITY				intensity = 0.f;
#	define RAW_COPY_INTENSITY				intensity = p.intensity;
#else
#	define RAW_ADD_INTENSITY
#	define RAW_REGISTER_INTENSITY
#	define RAW_ISFINITE_INTENSITY
#	define RAW_INIT_INTENSITY
#	define RAW_COPY_INTENSITY
#endif

#ifdef OUTPUT_PERPOINT_INTENSITY
#	define REC_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define REC_REGISTER_INTENSITY			(float, intensity, intensity)
#	define REC_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define REC_INIT_INTENSITY				intensity = 0.f;
#	define REC_COPY_INTENSITY				intensity = p.intensity;
#else
#	define REC_ADD_INTENSITY
#	define REC_REGISTER_INTENSITY
#	define REC_ISFINITE_INTENSITY
#	define REC_INIT_INTENSITY
#	define REC_COPY_INTENSITY
#endif

#ifdef PERPOINT_INTENSITY
#	define MED_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define MED_REGISTER_INTENSITY			(float, intensity, intensity)
#	define MED_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define MED_INIT_INTENSITY				intensity = 0.f;
#	define MED_COPY_INTENSITY				intensity = p.intensity;
#else
#	define MED_ADD_INTENSITY
#	define MED_REGISTER_INTENSITY
#	define MED_ISFINITE_INTENSITY
#	define MED_INIT_INTENSITY
#	define MED_COPY_INTENSITY
#endif

	// Normal
#ifdef INPUT_PERPOINT_NORMAL
#	define RAW_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define RAW_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define RAW_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define RAW_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define RAW_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;							
#else
#	define RAW_ADD_NORMAL
#	define RAW_REGISTER_NORMAL
#	define RAW_ISFINITE_NORMAL
#	define RAW_INIT_NORMAL
#	define RAW_COPY_NORMAL
#endif

#ifdef OUTPUT_PERPOINT_NORMAL
#	define REC_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define REC_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define REC_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define REC_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define REC_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#else
#	define REC_ADD_NORMAL
#	define REC_REGISTER_NORMAL
#	define REC_ISFINITE_NORMAL
#	define REC_INIT_NORMAL
#	define REC_COPY_NORMAL
#endif

#ifdef PERPOINT_NORMAL
#	define MED_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define MED_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define MED_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define MED_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define MED_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#else
#	define MED_ADD_NORMAL
#	define MED_REGISTER_NORMAL
#	define MED_ISFINITE_NORMAL
#	define MED_INIT_NORMAL
#	define MED_COPY_NORMAL
#endif

// Sharpness
#ifdef OUTPUT_PERPOINT_SHARPNESS
#	define REC_ADD_SHARPNESS				float sharpness; float specularIntensity;
#	define REC_REGISTER_SHARPNESS			(float, sharpness, sharpness) (float, specularIntensity, specularIntensity)
#	define REC_ISFINITE_SHARPNESS			&& pcl_isfinite(p.sharpness) && pcl_isfinite(p.specularIntensity)
#	define REC_INIT_SHARPNESS				sharpness = 0.f; specularIntensity = 1.f;
#	define REC_COPY_SHARPNESS				sharpness = p.sharpness; specularIntensity = p.specularIntensity;
#else
#	define REC_ADD_SHARPNESS
#	define REC_REGISTER_SHARPNESS
#	define REC_ISFINITE_SHARPNESS
#	define REC_INIT_SHARPNESS
#	define REC_COPY_SHARPNESS
#endif

#ifdef PERPOINT_SHARPNESS
#	define MED_ADD_SHARPNESS				float sharpness; float specularIntensity;
#	define MED_REGISTER_SHARPNESS			(float, sharpness, sharpness) (float, specularIntensity, specularIntensity)
#	define MED_ISFINITE_SHARPNESS			&& pcl_isfinite(p.sharpness) && pcl_isfinite(p.specularIntensity)
#	define MED_INIT_SHARPNESS				sharpness = 0.f; specularIntensity = 1.f;
#	define MED_COPY_SHARPNESS				sharpness = p.sharpness; specularIntensity = p.specularIntensity;
#else
#	define MED_ADD_SHARPNESS
#	define MED_REGISTER_SHARPNESS
#	define MED_ISFINITE_SHARPNESS
#	define MED_INIT_SHARPNESS
#	define MED_COPY_SHARPNESS
#endif

// Scan serial number
#ifdef INPUT_PERPOINT_SERIAL_NUMBER
#	define RAW_ADD_SERIAL_NUMBER			union { uint32_t serialNumber; int32_t hasSerialNumber; };
#	define RAW_REGISTER_SERIAL_NUMBER		(uint32_t, serialNumber, serialNumber)
#	define RAW_ISFINITE_SERIAL_NUMBER
#	define RAW_INIT_SERIAL_NUMBER			hasSerialNumber = -1;
#	define RAW_COPY_SERIAL_NUMBER			serialNumber = p.serialNumber;
#	define RAW_HAS_SERIAL_NUMBER			inline bool HasSerialNumber() const { return (hasSerialNumber != -1); }
#else
#	define RAW_ADD_SERIAL_NUMBER
#	define RAW_REGISTER_SERIAL_NUMBER
#	define RAW_ISFINITE_SERIAL_NUMBER
#	define RAW_INIT_SERIAL_NUMBER
#	define RAW_COPY_SERIAL_NUMBER
#	define RAW_HAS_SERIAL_NUMBER
#endif

#ifdef PERPOINT_SERIAL_NUMBER
#	define MED_ADD_SERIAL_NUMBER			union { uint32_t serialNumber; int32_t hasSerialNumber; };
#	define MED_REGISTER_SERIAL_NUMBER		(uint32_t, serialNumber, serialNumber)
#	define MED_ISFINITE_SERIAL_NUMBER
#	define MED_INIT_SERIAL_NUMBER			hasSerialNumber = -1;
#	define MED_COPY_SERIAL_NUMBER			serialNumber = p.serialNumber;
#	define MED_HAS_SERIAL_NUMBER			inline bool HasSerialNumber() const { return (hasSerialNumber != -1); }
#else
#	define MED_ADD_SERIAL_NUMBER
#	define MED_REGISTER_SERIAL_NUMBER
#	define MED_ISFINITE_SERIAL_NUMBER
#	define MED_INIT_SERIAL_NUMBER
#	define MED_COPY_SERIAL_NUMBER
#	define MED_HAS_SERIAL_NUMBER
#endif

// Label
#ifdef OUTPUT_PERPOINT_LABEL
#	define REC_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#	define REC_REGISTER_LABEL				(uint32_t, label, label)
#	define REC_ISFINITE_LABEL
#	define REC_INIT_LABEL					hasLabel = -1;
#	define REC_COPY_LABEL					label = p.label;
#	define REC_HAS_LABEL					inline bool HasLabel() const { return (hasLabel != -1); }
#else
#	define REC_ADD_LABEL
#	define REC_REGISTER_LABEL
#	define REC_ISFINITE_LABEL
#	define REC_INIT_LABEL
#	define REC_COPY_LABEL
#	define REC_HAS_LABEL
#endif

#ifdef PERPOINT_LABEL
#	define MED_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#	define MED_REGISTER_LABEL				(uint32_t, label, label)
#	define MED_ISFINITE_LABEL
#	define MED_INIT_LABEL					hasLabel = -1;
#	define MED_COPY_LABEL					label = p.label;
#	define MED_HAS_LABEL					inline bool HasLabel() const { return (hasLabel != -1); }
#else
#	define MED_ADD_LABEL
#	define MED_REGISTER_LABEL
#	define MED_ISFINITE_LABEL
#	define MED_INIT_LABEL
#	define MED_COPY_LABEL
#	define MED_HAS_LABEL
#endif

	//
	struct EIGEN_ALIGN16 _PointRAW
	{
		PCL_ADD_POINT4D;
		RAW_ADD_RGB;
		RAW_ADD_INTENSITY;
		RAW_ADD_NORMAL;
		RAW_ADD_SERIAL_NUMBER;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointREC
	{
		PCL_ADD_POINT4D;
		REC_ADD_RGB;
		REC_ADD_INTENSITY;
		REC_ADD_NORMAL;
		REC_ADD_SHARPNESS;
		REC_ADD_LABEL;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointMED
	{
		PCL_ADD_POINT4D;
		MED_ADD_RGB;
		MED_ADD_INTENSITY;
		MED_ADD_NORMAL;
		MED_ADD_SHARPNESS;
		MED_ADD_SERIAL_NUMBER;
		MED_ADD_LABEL;

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
			RAW_INIT_RGB;
			RAW_INIT_INTENSITY;
			RAW_INIT_NORMAL;
			RAW_INIT_SERIAL_NUMBER;
		}

		inline PointRAW(const PointRAW& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			RAW_COPY_RGB;
			RAW_COPY_INTENSITY;
			RAW_COPY_NORMAL;
			RAW_COPY_SERIAL_NUMBER;
		}

		inline PointRAW(const PointREC& p);
		inline PointRAW(const PointMED& p);

		inline PointRAW& operator = (const PointREC &p);
		inline PointRAW& operator = (const PointMED &p);

		RAW_HAS_SERIAL_NUMBER;
	};

	struct PointREC : public _PointREC
	{
		inline PointREC()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			REC_INIT_RGB;
			REC_INIT_INTENSITY;
			REC_INIT_NORMAL;
			REC_INIT_SHARPNESS;
			REC_INIT_LABEL;
		}

		inline PointREC(const PointREC& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			REC_COPY_RGB;
			REC_COPY_INTENSITY;
			REC_COPY_NORMAL;
			REC_COPY_SHARPNESS;
			REC_COPY_LABEL;
		}

		inline PointREC(const PointRAW& p);
		inline PointREC(const PointMED& p);

		inline PointREC& operator = (const PointRAW &p);
		inline PointREC& operator = (const PointMED &p);

		REC_HAS_LABEL;
	};

	struct PointMED : public _PointMED
	{
		inline PointMED()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			MED_INIT_RGB;
			MED_INIT_INTENSITY;
			MED_INIT_NORMAL;
			MED_INIT_SHARPNESS;
			MED_INIT_SERIAL_NUMBER;
			MED_INIT_LABEL;
		}

		inline PointMED(const PointMED& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			MED_COPY_RGB;
			MED_COPY_INTENSITY;
			MED_COPY_NORMAL;
			MED_COPY_SHARPNESS;
			MED_COPY_SERIAL_NUMBER;
			MED_COPY_LABEL;
		}

		inline PointMED(const PointRAW& p);
		inline PointMED(const PointREC& p);

		inline PointMED& operator = (const PointRAW &p);
		inline PointMED& operator = (const PointREC &p);

		MED_HAS_SERIAL_NUMBER; 
		MED_HAS_LABEL;
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

		inline PointNDF(float normalX, float normalY, float normalZ, uint32_t label_, float intensity_)
		{
			x = ((float)label_) + 0.5f; y = 0.5f; z = 0.5f; data[3] = 1.0f;
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
RAW_REGISTER_RGB
RAW_REGISTER_INTENSITY
RAW_REGISTER_NORMAL
RAW_REGISTER_SERIAL_NUMBER
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointRAW, RecRoom::PointRAW)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointREC,
(float, x, x) (float, y, y) (float, z, z)
REC_REGISTER_RGB
REC_REGISTER_INTENSITY
REC_REGISTER_NORMAL
REC_REGISTER_SHARPNESS
REC_REGISTER_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointREC, RecRoom::PointREC)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointMED,
(float, x, x) (float, y, y) (float, z, z)
MED_REGISTER_RGB
MED_REGISTER_INTENSITY
MED_REGISTER_NORMAL
MED_REGISTER_SHARPNESS
MED_REGISTER_SERIAL_NUMBER
MED_REGISTER_LABEL
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