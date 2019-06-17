#pragma once

#include <pcl/point_types.h>

#include "Common.h"

namespace RecRoom
{
	// Macros

	// Normal
#ifdef POINT_E57_WITH_NORMAL
#define E57_CAN_CONTAIN_NORMAL			true
#define E57_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#define E57_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#define E57_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#define E57_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#define E57_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;							
#else
#define E57_CAN_CONTAIN_NORMAL			false
#define E57_ADD_NORMAL
#define E57_REGISTER_NORMAL
#define E57_ISFINITE_NORMAL
#define E57_INIT_NORMAL
#define E57_COPY_NORMAL
#endif

#ifdef POINT_PCD_WITH_NORMAL
#define PCD_CAN_CONTAIN_NORMAL			true
#define PCD_ADD_NORMAL					PCL_ADD_NORMAL4D; float curvature;
#define PCD_REGISTER_NORMAL				(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#define PCD_ISFINITE_NORMAL				&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#define PCD_INIT_NORMAL					normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#define PCD_COPY_NORMAL					normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#else
#define PCD_CAN_CONTAIN_NORMAL			false
#define PCD_ADD_NORMAL
#define PCD_REGISTER_NORMAL
#define PCD_ISFINITE_NORMAL
#define PCD_INIT_NORMAL
#define PCD_COPY_NORMAL
#endif

#ifdef POINT_E57_WITH_NORMAL
#define EXCHANGE_CAN_CONTAIN_NORMAL		true
#define EXCHANGE_ADD_NORMAL				PCL_ADD_NORMAL4D; float curvature;
#define EXCHANGE_REGISTER_NORMAL		(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#define EXCHANGE_ISFINITE_NORMAL		&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#define EXCHANGE_INIT_NORMAL			normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#define EXCHANGE_COPY_NORMAL			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#elif defined POINT_PCD_WITH_NORMAL
#define EXCHANGE_CAN_CONTAIN_NORMAL		true
#define EXCHANGE_ADD_NORMAL				PCL_ADD_NORMAL4D; float curvature;
#define EXCHANGE_REGISTER_NORMAL		(float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#define EXCHANGE_ISFINITE_NORMAL		&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z)
#define EXCHANGE_INIT_NORMAL			normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#define EXCHANGE_COPY_NORMAL			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#else
#define EXCHANGE_CAN_CONTAIN_NORMAL		false
#define EXCHANGE_ADD_NORMAL
#define EXCHANGE_REGISTER_NORMAL
#define EXCHANGE_ISFINITE_NORMAL
#define EXCHANGE_INIT_NORMAL
#define EXCHANGE_COPY_NORMAL
#endif

// RGB
#ifdef POINT_E57_WITH_RGB
#define E57_CAN_CONTAIN_RGB				true
#define E57_ADD_RGB						PCL_ADD_RGB;
#define E57_REGISTER_RGB				(uint32_t, rgba, rgba)
#define E57_ISFINITE_RGB
#define E57_INIT_RGB					r = g = b = 0; a = 1;
#define E57_COPY_RGB					rgba = p.rgba;
#else
#define E57_CAN_CONTAIN_RGB				false
#define E57_ADD_RGB
#define E57_REGISTER_RGB
#define E57_ISFINITE_RGB
#define E57_INIT_RGB
#define E57_COPY_RGB
#endif

#ifdef POINT_PCD_WITH_RGB
#define PCD_CAN_CONTAIN_RGB				true
#define PCD_ADD_RGB						PCL_ADD_RGB;
#define PCD_REGISTER_RGB				(uint32_t, rgba, rgba)
#define PCD_ISFINITE_RGB
#define PCD_INIT_RGB					r = g = b = 0; a = 1;
#define PCD_COPY_RGB					rgba = p.rgba;
#else
#define PCD_CAN_CONTAIN_RGB				false
#define PCD_ADD_RGB
#define PCD_REGISTER_RGB
#define PCD_ISFINITE_RGB
#define PCD_INIT_RGB
#define PCD_COPY_RGB
#endif

#ifdef POINT_E57_WITH_RGB
#define EXCHANGE_CAN_CONTAIN_RGB		true
#define EXCHANGE_ADD_RGB				PCL_ADD_RGB;
#define EXCHANGE_REGISTER_RGB			(uint32_t, rgba, rgba)
#define EXCHANGE_ISFINITE_RGB
#define EXCHANGE_INIT_RGB				r = g = b = 0; a = 1;
#define EXCHANGE_COPY_RGB				rgba = p.rgba;
#elif defined POINT_PCD_WITH_RGB
#define EXCHANGE_CAN_CONTAIN_RGB		true
#define EXCHANGE_ADD_RGB				PCL_ADD_RGB;
#define EXCHANGE_REGISTER_RGB			(uint32_t, rgba, rgba)
#define EXCHANGE_ISFINITE_RGB
#define EXCHANGE_INIT_RGB				r = g = b = 0; a = 1;
#define EXCHANGE_COPY_RGB				rgba = p.rgba;
#else
#define EXCHANGE_CAN_CONTAIN_RGB		false
#define EXCHANGE_ADD_RGB
#define EXCHANGE_REGISTER_RGB
#define EXCHANGE_ISFINITE_RGB
#define EXCHANGE_INIT_RGB
#define EXCHANGE_COPY_RGB
#endif

// Intensity
#ifdef POINT_E57_WITH_INTENSITY
#define E57_CAN_CONTAIN_INTENSITY		true
#define E57_ADD_INTENSITY				PCL_ADD_INTENSITY;
#define E57_REGISTER_INTENSITY			(float, intensity, intensity)
#define E57_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#define E57_INIT_INTENSITY				intensity = 0.f;
#define E57_COPY_INTENSITY				intensity = p.intensity;
#else
#define E57_CAN_CONTAIN_INTENSITY		false
#define E57_ADD_INTENSITY
#define E57_REGISTER_INTENSITY
#define E57_ISFINITE_INTENSITY
#define E57_INIT_INTENSITY
#define E57_COPY_INTENSITY
#endif

#ifdef POINT_PCD_WITH_INTENSITY
#define PCD_CAN_CONTAIN_INTENSITY		true
#define PCD_ADD_INTENSITY				PCL_ADD_INTENSITY;
#define PCD_REGISTER_INTENSITY			(float, intensity, intensity)
#define PCD_ISFINITE_INTENSITY			&& pcl_isfinite(p.intensity)
#define PCD_INIT_INTENSITY				intensity = 0.f;
#define PCD_COPY_INTENSITY				intensity = p.intensity;
#else
#define PCD_CAN_CONTAIN_INTENSITY		false
#define PCD_ADD_INTENSITY
#define PCD_REGISTER_INTENSITY
#define PCD_ISFINITE_INTENSITY
#define PCD_INIT_INTENSITY
#define PCD_COPY_INTENSITY
#endif

#ifdef POINT_E57_WITH_INTENSITY
#define EXCHANGE_CAN_CONTAIN_INTENSITY	true
#define EXCHANGE_ADD_INTENSITY			PCL_ADD_INTENSITY;
#define EXCHANGE_REGISTER_INTENSITY		(float, intensity, intensity)
#define EXCHANGE_ISFINITE_INTENSITY		&& pcl_isfinite(p.intensity)
#define EXCHANGE_INIT_INTENSITY			intensity = 0.f;
#define EXCHANGE_COPY_INTENSITY			intensity = p.intensity;
#elif defined POINT_PCD_WITH_INTENSITY
#define EXCHANGE_CAN_CONTAIN_INTENSITY	true
#define EXCHANGE_ADD_INTENSITY			PCL_ADD_INTENSITY;
#define EXCHANGE_REGISTER_INTENSITY		(float, intensity, intensity)
#define EXCHANGE_ISFINITE_INTENSITY		&& pcl_isfinite(p.intensity)
#define EXCHANGE_INIT_INTENSITY			intensity = 0.f;
#define EXCHANGE_COPY_INTENSITY			intensity = p.intensity;
#else
#define EXCHANGE_CAN_CONTAIN_INTENSITY	false
#define EXCHANGE_ADD_INTENSITY
#define EXCHANGE_REGISTER_INTENSITY
#define EXCHANGE_ISFINITE_INTENSITY
#define EXCHANGE_INIT_INTENSITY
#define EXCHANGE_COPY_INTENSITY
#endif

// Label
#ifdef POINT_E57_WITH_LABEL
#define E57_CAN_CONTAIN_LABEL			true
#define E57_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#define E57_REGISTER_LABEL				(uint32_t, label, label)
#define E57_ISFINITE_LABEL
#define E57_INIT_LABEL					hasLabel = -1;
#define E57_COPY_LABEL					label = p.label;
#define E57_HASLABEL					inline bool HasLabel() { return (hasLabel != -1); }
#else
#define E57_CAN_CONTAIN_LABEL			false
#define E57_ADD_LABEL
#define E57_REGISTER_LABEL
#define E57_ISFINITE_LABEL
#define E57_INIT_LABEL
#define E57_COPY_LABEL
#define E57_HASLABEL
#endif

#ifdef POINT_PCD_WITH_LABEL
#define PCD_CAN_CONTAIN_LABEL			true
#define PCD_ADD_LABEL					union { uint32_t label; int32_t hasLabel; };
#define PCD_REGISTER_LABEL				(uint32_t, label, label)
#define PCD_ISFINITE_LABEL
#define PCD_INIT_LABEL					hasLabel = -1;
#define PCD_COPY_LABEL					label = p.label;
#define PCD_COPY_SEGLABEL				label = p.segLabel;
#define PCD_HASLABEL					inline bool HasLabel() { return (hasLabel != -1); }
#else
#define PCD_CAN_CONTAIN_LABEL			false
#define PCD_ADD_LABEL
#define PCD_REGISTER_LABEL
#define PCD_ISFINITE_LABEL
#define PCD_INIT_LABEL
#define PCD_COPY_LABEL
#define PCD_COPY_SEGLABEL
#define PCD_HASLABEL
#endif

#ifdef POINT_E57_WITH_LABEL
#define EXCHANGE_CAN_CONTAIN_LABEL		true
#define EXCHANGE_ADD_LABEL				union { uint32_t label; int32_t hasLabel; };
#define EXCHANGE_REGISTER_LABEL			(uint32_t, label, label)
#define EXCHANGE_ISFINITE_LABEL
#define EXCHANGE_INIT_LABEL				hasLabel = -1;
#define EXCHANGE_COPY_LABEL				label = p.label;
#define EXCHANGE_HASLABEL				inline bool HasLabel() { return (hasLabel != -1); }
#elif defined POINT_PCD_WITH_LABEL
#define EXCHANGE_CAN_CONTAIN_LABEL		true
#define EXCHANGE_ADD_LABEL				union { uint32_t label; int32_t hasLabel; };
#define EXCHANGE_REGISTER_LABEL			(uint32_t, label, label)
#define EXCHANGE_ISFINITE_LABEL
#define EXCHANGE_INIT_LABEL
#define EXCHANGE_COPY_LABEL
#define EXCHANGE_HASLABEL
#else
#define EXCHANGE_CAN_CONTAIN_LABEL		false
#define EXCHANGE_ADD_LABEL
#define EXCHANGE_REGISTER_LABEL
#define EXCHANGE_ISFINITE_LABEL
#define EXCHANGE_INIT_LABEL
#define EXCHANGE_COPY_LABEL
#define EXCHANGE_HASLABEL
#endif

#ifdef POINT_PCD_WITH_LABEL
#define EXCHANGE_ADD_SEGLABEL			union { uint32_t segLabel; int32_t hasSegLabel; };
#define EXCHANGE_REGISTER_SEGLABEL		(uint32_t, segLabel, segLabel)
#define EXCHANGE_ISFINITE_SEGLABEL
#define EXCHANGE_INIT_SEGLABEL			hasSegLabel = -1;
#define EXCHANGE_COPY_SEGLABEL			segLabel = p.segLabel;
#define EXCHANGE_HASSEGLABEL			inline bool HasSegLabel() { return (hasSegLabel != -1); }
#else
#define EXCHANGE_ADD_SEGLABEL
#define EXCHANGE_REGISTER_SEGLABEL
#define EXCHANGE_ISFINITE_SEGLABEL
#define EXCHANGE_INIT_SEGLABEL
#define EXCHANGE_COPY_SEGLABEL
#define EXCHANGE_HASSEGLABEL
#endif

	//
	struct EIGEN_ALIGN16 _PointE57
	{
		PCL_ADD_POINT4D;
		E57_ADD_NORMAL;
		E57_ADD_RGB;
		E57_ADD_INTENSITY;
		E57_ADD_LABEL;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointPCD
	{
		PCL_ADD_POINT4D;
		PCD_ADD_NORMAL;
		PCD_ADD_RGB;
		PCD_ADD_INTENSITY;
		PCD_ADD_LABEL;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointExchange
	{
		PCL_ADD_POINT4D;
		EXCHANGE_ADD_NORMAL;
		EXCHANGE_ADD_RGB;
		EXCHANGE_ADD_INTENSITY;
		EXCHANGE_ADD_LABEL;
		EXCHANGE_ADD_SEGLABEL;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct EIGEN_ALIGN16 _PointNDF
	{
		PCL_ADD_POINT4D;
		PCL_ADD_INTENSITY;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	//
	struct PointExchange;
	struct PointE57;
	struct PointPCD;

	struct PointE57 : public _PointE57
	{
		inline PointE57()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			E57_INIT_NORMAL;
			E57_INIT_RGB;
			E57_INIT_INTENSITY;
			E57_INIT_LABEL;
		}

		inline PointE57(const PointE57& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			E57_COPY_NORMAL;
			E57_COPY_RGB;
			E57_COPY_INTENSITY;
			E57_COPY_LABEL;
		}

		inline PointE57(const PointPCD& p);
		inline PointE57(const PointExchange& p);
		
		E57_HASLABEL;
	};

	struct PointPCD : public _PointPCD
	{
		inline PointPCD()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			PCD_INIT_NORMAL;
			PCD_INIT_RGB;
			PCD_INIT_INTENSITY;
			PCD_INIT_LABEL;
		}

		inline PointPCD(const PointPCD& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			PCD_COPY_NORMAL;
			PCD_COPY_RGB;
			PCD_COPY_INTENSITY;
			PCD_COPY_LABEL;
		}

		inline PointPCD(const PointE57& p);
		inline PointPCD(const PointExchange& p);

		PCD_HASLABEL;
	};

	struct PointExchange : public _PointExchange
	{
		inline PointExchange()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			EXCHANGE_INIT_NORMAL;
			EXCHANGE_INIT_RGB;
			EXCHANGE_INIT_INTENSITY;
			EXCHANGE_INIT_LABEL;
			EXCHANGE_INIT_SEGLABEL;
		}

		inline PointExchange(const PointExchange& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			EXCHANGE_COPY_NORMAL;
			EXCHANGE_COPY_RGB;
			EXCHANGE_COPY_INTENSITY;
			EXCHANGE_COPY_LABEL;
			EXCHANGE_COPY_SEGLABEL;
		}

		inline PointExchange(const PointE57& p);
		inline PointExchange(const PointPCD& p);

		EXCHANGE_HASLABEL;
		EXCHANGE_HASSEGLABEL;
	};


	struct PointNDF : public _PointNDF
	{
		inline PointNDF()
		{
			x = y = z = 0.0f; data[3] = 1.f;
			intensity = 0.f;
		}

		inline PointNDF(const PointNDF& p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
		}
	};
}

//
POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointE57,
(float, x, x) (float, y, y) (float, z, z)
E57_REGISTER_NORMAL
E57_REGISTER_RGB
E57_REGISTER_INTENSITY
E57_REGISTER_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointE57, RecRoom::PointE57)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointPCD,
(float, x, x) (float, y, y) (float, z, z)
PCD_REGISTER_NORMAL
PCD_REGISTER_RGB
PCD_REGISTER_INTENSITY
PCD_REGISTER_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointPCD, RecRoom::PointPCD)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointExchange,
(float, x, x) (float, y, y) (float, z, z)
EXCHANGE_REGISTER_NORMAL
EXCHANGE_REGISTER_RGB
EXCHANGE_REGISTER_INTENSITY
EXCHANGE_REGISTER_LABEL
EXCHANGE_REGISTER_SEGLABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointExchange, RecRoom::PointExchange)

POINT_CLOUD_REGISTER_POINT_STRUCT(RecRoom::PointNDF,
(float, x, x) (float, y, y) (float, z, z)
(float, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(RecRoom::PointNDF, RecRoom::PointNDF)

#include "PointType.hpp"