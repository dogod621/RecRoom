#pragma once

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include "Common.h"

namespace RecRoom
{
	// Macros

	// Normal
#ifdef POINT_RAW_WITH_NORMAL
#	define RAW_CAN_CONTAIN_NORMAL			true
#	define RAW_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define RAW_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define RAW_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define RAW_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define RAW_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;							
#else
#	define RAW_CAN_CONTAIN_NORMAL			false
#	define RAW_ADD_NORMAL
#	define RAW_REGISTER_NORMAL
#	define RAW_ISFINITE_NORMAL
#	define RAW_INIT_NORMAL
#	define RAW_COPY_NORMAL
#endif

#ifdef POINT_REC_WITH_NORMAL
#	define REC_CAN_CONTAIN_NORMAL			true
#	define REC_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define REC_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define REC_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define REC_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define REC_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#else
#	define REC_CAN_CONTAIN_NORMAL			false
#	define REC_ADD_NORMAL
#	define REC_REGISTER_NORMAL
#	define REC_ISFINITE_NORMAL
#	define REC_INIT_NORMAL
#	define REC_COPY_NORMAL
#endif

#ifdef POINT_RAW_WITH_NORMAL
#	define POINT_MED_WITH_NORMAL
#	define MED_CAN_CONTAIN_NORMAL			true
#	define MED_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define MED_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define MED_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define MED_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define MED_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#elif defined POINT_REC_WITH_NORMAL
#	define POINT_MED_WITH_NORMAL
#	define MED_CAN_CONTAIN_NORMAL			true
#	define MED_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#	define MED_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#	define MED_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#	define MED_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#	define MED_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#else
#	define MED_CAN_CONTAIN_NORMAL			false
#	define MED_ADD_NORMAL
#	define MED_REGISTER_NORMAL
#	define MED_ISFINITE_NORMAL
#	define MED_INIT_NORMAL
#	define MED_COPY_NORMAL
#endif

// RGB
#ifdef POINT_RAW_WITH_RGB
#	define RAW_CAN_CONTAIN_RGB				true
#	define RAW_ADD_RGB						PCL_ADD_RGB;
#	define RAW_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define RAW_ISFINITE_RGB
#	define RAW_INIT_RGB						r = g = b = 0; a = 1;
#	define RAW_COPY_RGB						rgba = p.rgba;
#else
#	define RAW_CAN_CONTAIN_RGB				false
#	define RAW_ADD_RGB
#	define RAW_REGISTER_RGB
#	define RAW_ISFINITE_RGB
#	define RAW_INIT_RGB
#	define RAW_COPY_RGB
#endif

#ifdef POINT_REC_WITH_RGB
#	define REC_CAN_CONTAIN_RGB				true
#	define REC_ADD_RGB						PCL_ADD_RGB;
#	define REC_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define REC_ISFINITE_RGB
#	define REC_INIT_RGB						r = g = b = 0; a = 1;
#	define REC_COPY_RGB						rgba = p.rgba;
#else
#	define REC_CAN_CONTAIN_RGB				false
#	define REC_ADD_RGB
#	define REC_REGISTER_RGB
#	define REC_ISFINITE_RGB
#	define REC_INIT_RGB
#	define REC_COPY_RGB
#endif

#ifdef POINT_RAW_WITH_RGB
#	define POINT_MED_WITH_RGB
#	define MED_CAN_CONTAIN_RGB				true
#	define MED_ADD_RGB						PCL_ADD_RGB;
#	define MED_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define MED_ISFINITE_RGB
#	define MED_INIT_RGB						r = g = b = 0; a = 1;
#	define MED_COPY_RGB						rgba = p.rgba;
#elif defined POINT_REC_WITH_RGB
#	define POINT_MED_WITH_RGB
#	define MED_CAN_CONTAIN_RGB				true
#	define MED_ADD_RGB						PCL_ADD_RGB;
#	define MED_REGISTER_RGB					(uint32_t, rgba, rgba)
#	define MED_ISFINITE_RGB
#	define MED_INIT_RGB						r = g = b = 0; a = 1;
#	define MED_COPY_RGB						rgba = p.rgba;
#else
#	define MED_CAN_CONTAIN_RGB				false
#	define MED_ADD_RGB
#	define MED_REGISTER_RGB
#	define MED_ISFINITE_RGB
#	define MED_INIT_RGB
#	define MED_COPY_RGB
#endif

// Intensity
#ifdef POINT_RAW_WITH_INTENSITY
#	define RAW_CAN_CONTAIN_INTENSITY		true
#	define RAW_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define RAW_REGISTER_INTENSITY			(float, intensity, intensity)
#	define RAW_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define RAW_INIT_INTENSITY				intensity = 0.f;
#	define RAW_COPY_INTENSITY				intensity = p.intensity;
#else
#	define RAW_CAN_CONTAIN_INTENSITY		false
#	define RAW_ADD_INTENSITY
#	define RAW_REGISTER_INTENSITY
#	define RAW_ISFINITE_INTENSITY
#	define RAW_INIT_INTENSITY
#	define RAW_COPY_INTENSITY
#endif

#ifdef POINT_REC_WITH_INTENSITY
#	define REC_CAN_CONTAIN_INTENSITY		true
#	define REC_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define REC_REGISTER_INTENSITY			(float, intensity, intensity)
#	define REC_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define REC_INIT_INTENSITY				intensity = 0.f;
#	define REC_COPY_INTENSITY				intensity = p.intensity;
#else
#	define REC_CAN_CONTAIN_INTENSITY		false
#	define REC_ADD_INTENSITY
#	define REC_REGISTER_INTENSITY
#	define REC_ISFINITE_INTENSITY
#	define REC_INIT_INTENSITY
#	define REC_COPY_INTENSITY
#endif

#ifdef POINT_RAW_WITH_INTENSITY
#	define POINT_MED_WITH_INTENSITY
#	define MED_CAN_CONTAIN_INTENSITY		true
#	define MED_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define MED_REGISTER_INTENSITY			(float, intensity, intensity)
#	define MED_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define MED_INIT_INTENSITY				intensity = 0.f;
#	define MED_COPY_INTENSITY				intensity = p.intensity;
#elif defined POINT_REC_WITH_INTENSITY
#	define POINT_MED_WITH_INTENSITY
#	define MED_CAN_CONTAIN_INTENSITY		true
#	define MED_ADD_INTENSITY				PCL_ADD_INTENSITY;
#	define MED_REGISTER_INTENSITY			(float, intensity, intensity)
#	define MED_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#	define MED_INIT_INTENSITY				intensity = 0.f;
#	define MED_COPY_INTENSITY				intensity = p.intensity;
#else
#	define MED_CAN_CONTAIN_INTENSITY		false
#	define MED_ADD_INTENSITY
#	define MED_REGISTER_INTENSITY
#	define MED_ISFINITE_INTENSITY
#	define MED_INIT_INTENSITY
#	define MED_COPY_INTENSITY
#endif

// Label
#ifdef POINT_RAW_WITH_LABEL
#	define RAW_CAN_CONTAIN_LABEL			true
#	define RAW_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#	define RAW_REGISTER_LABEL				(uint32_t, label, label)
#	define RAW_ISFINITE_LABEL
#	define RAW_INIT_LABEL					hasLabel = -1;
#	define RAW_COPY_LABEL					label = p.label;
#	define RAW_HASLABEL						inline bool HasLabel() { return (hasLabel != -1); }
#else
#	define RAW_CAN_CONTAIN_LABEL			false
#	define RAW_ADD_LABEL
#	define RAW_REGISTER_LABEL
#	define RAW_ISFINITE_LABEL
#	define RAW_INIT_LABEL
#	define RAW_COPY_LABEL
#	define RAW_HASLABEL
#endif

#ifdef POINT_REC_WITH_LABEL
#	define REC_CAN_CONTAIN_LABEL			true
#	define REC_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#	define REC_REGISTER_LABEL				(uint32_t, label, label)
#	define REC_ISFINITE_LABEL
#	define REC_INIT_LABEL					hasLabel = -1;
#	define REC_COPY_LABEL					label = p.label;
#	define REC_COPY_SEGLABEL				label = p.segLabel;
#	define REC_HASLABEL						inline bool HasLabel() { return (hasLabel != -1); }
#else
#	define REC_CAN_CONTAIN_LABEL			false
#	define REC_ADD_LABEL
#	define REC_REGISTER_LABEL
#	define REC_ISFINITE_LABEL
#	define REC_INIT_LABEL
#	define REC_COPY_LABEL
#	define REC_COPY_SEGLABEL
#	define REC_HASLABEL
#endif

#ifdef POINT_RAW_WITH_LABEL
#	define POINT_MED_WITH_LABEL
#	define MED_CAN_CONTAIN_LABEL			true
#	define MED_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#	define MED_REGISTER_LABEL				(uint32_t, label, label)
#	define MED_ISFINITE_LABEL
#	define MED_INIT_LABEL					hasLabel = -1;
#	define MED_COPY_LABEL					label = p.label;
#	define MED_HASLABEL						inline bool HasLabel() { return (hasLabel != -1); }
#elif defined POINT_REC_WITH_LABEL
#	define POINT_MED_WITH_LABEL
#	define MED_CAN_CONTAIN_LABEL			true
#	define MED_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#	define MED_REGISTER_LABEL				(uint32_t, label, label)
#	define MED_ISFINITE_LABEL
#	define MED_INIT_LABEL
#	define MED_COPY_LABEL
#	define MED_HASLABEL
#else
#	define MED_CAN_CONTAIN_LABEL			false
#	define MED_ADD_LABEL
#	define MED_REGISTER_LABEL
#	define MED_ISFINITE_LABEL
#	define MED_INIT_LABEL
#	define MED_COPY_LABEL
#	define MED_HASLABEL
#endif

#ifdef POINT_REC_WITH_LABEL
#	define POINT_MED_WITH_SEGLABEL
#	define MED_ADD_SEGLABEL					union { uint32_t segLabel; int32_t hasSegLabel; };
#	define MED_REGISTER_SEGLABEL			(uint32_t, segLabel, segLabel)
#	define MED_ISFINITE_SEGLABEL
#	define MED_INIT_SEGLABEL				hasSegLabel = -1;
#	define MED_COPY_SEGLABEL				segLabel = p.segLabel;
#	define MED_HASSEGLABEL					inline bool HasSegLabel() { return (hasSegLabel != -1); }
#else
#	define POINT_MED_WITH_SEGLABEL
#	define MED_ADD_SEGLABEL
#	define MED_REGISTER_SEGLABEL
#	define MED_ISFINITE_SEGLABEL
#	define MED_INIT_SEGLABEL
#	define MED_COPY_SEGLABEL
#	define MED_HASSEGLABEL
#endif

	//
	struct EIGEN_ALIGN16 _PointRAW
	{
		PCL_ADD_POINT4D;
		RAW_ADD_NORMAL;
		RAW_ADD_RGB;
		RAW_ADD_INTENSITY;
		RAW_ADD_LABEL;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointREC
	{
		PCL_ADD_POINT4D;
		REC_ADD_NORMAL;
		REC_ADD_RGB;
		REC_ADD_INTENSITY;
		REC_ADD_LABEL;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointMED
	{
		PCL_ADD_POINT4D;
		MED_ADD_NORMAL;
		MED_ADD_RGB;
		MED_ADD_INTENSITY;
		MED_ADD_LABEL;
		MED_ADD_SEGLABEL;

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
			RAW_INIT_NORMAL;
			RAW_INIT_RGB;
			RAW_INIT_INTENSITY;
			RAW_INIT_LABEL;
		}

		inline PointRAW(const PointRAW& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			RAW_COPY_NORMAL;
			RAW_COPY_RGB;
			RAW_COPY_INTENSITY;
			RAW_COPY_LABEL;
		}

		inline PointRAW(const PointREC& p);
		inline PointRAW(const PointMED& p);
		
		RAW_HASLABEL;
	};

	struct PointREC : public _PointREC
	{
		inline PointREC()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			REC_INIT_NORMAL;
			REC_INIT_RGB;
			REC_INIT_INTENSITY;
			REC_INIT_LABEL;
		}

		inline PointREC(const PointREC& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			REC_COPY_NORMAL;
			REC_COPY_RGB;
			REC_COPY_INTENSITY;
			REC_COPY_LABEL;
		}

		inline PointREC(const PointRAW& p);
		inline PointREC(const PointMED& p);

		REC_HASLABEL;
	};

	struct PointMED : public _PointMED
	{
		inline PointMED()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			MED_INIT_NORMAL;
			MED_INIT_RGB;
			MED_INIT_INTENSITY;
			MED_INIT_LABEL;
			MED_INIT_SEGLABEL;
		}

		inline PointMED(const PointMED& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			MED_COPY_NORMAL;
			MED_COPY_RGB;
			MED_COPY_INTENSITY;
			MED_COPY_LABEL;
			MED_COPY_SEGLABEL;
		}

		inline PointMED(const PointRAW& p);
		inline PointMED(const PointREC& p);

		MED_HASLABEL;
		MED_HASSEGLABEL;
	};

	struct PointNDF : public _PointNDF
	{
		inline PointNDF()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			normal_x = normal_y = normal_z = data_n[3];
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
			normal_x = normal_y = normal_z = data_n[3];
			intensity = 0.f;
		}

		inline PointLF(const PointLF& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f;
			intensity = p.intensity;
		}
	};

	using PcIndex = std::vector<int>;
	using PcRAW = Pc<PointRAW>;
	using PcMED = Pc<PointMED>;
	using PcREC = Pc<PointREC>;
	using PcNDF = Pc<PointNDF>;
	using PcLF = Pc<PointLF>;

	using AccRAW = pcl::search::Search<PointRAW>;
	using AccMED = pcl::search::Search<PointMED>;
	using AccREC = pcl::search::Search<PointREC>;
	using AccNDF = pcl::search::Search<PointNDF>;
	using AccLF = pcl::search::Search<PointLF>;

	using KdTreeRAW = pcl::search::KdTree<PointRAW>;
	using KdTreeMED = pcl::search::KdTree<PointMED>;
	using KdTreeREC = pcl::search::KdTree<PointREC>;
	using KdTreeNDF = pcl::search::KdTree<PointNDF>;
	using KdTreeLF = pcl::search::KdTree<PointLF>;
}

//
POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointRAW,
(float, x, x) (float, y, y) (float, z, z)
RAW_REGISTER_NORMAL
RAW_REGISTER_RGB
RAW_REGISTER_INTENSITY
RAW_REGISTER_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointRAW, RecRoom::PointRAW)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointREC,
(float, x, x) (float, y, y) (float, z, z)
REC_REGISTER_NORMAL
REC_REGISTER_RGB
REC_REGISTER_INTENSITY
REC_REGISTER_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointREC, RecRoom::PointREC)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointMED,
(float, x, x) (float, y, y) (float, z, z)
MED_REGISTER_NORMAL
MED_REGISTER_RGB
MED_REGISTER_INTENSITY
MED_REGISTER_LABEL
MED_REGISTER_SEGLABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointMED, RecRoom::PointMED)

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