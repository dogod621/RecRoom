#pragma once

#include "Common.h"

namespace RecRoom
{
#define AXIS_NUM_BITS 3
#define AXIS_2_NUM_BITS 6
	enum Axis : Flag
	{
		Axis_UNKNOWN = 0,

		PX = 1, // Positive X axis.
		PY = 2, // Positive Y axis.
		PZ = 3, // Positive Z axis.

		NX = 4, // Negative X axis.
		NY = 5, // Negative Y axis.
		NZ = 6, // Negative Z axis.
	};

#define UVMode_NUM_BITS 16
	enum UVMode : Flag
	{
		UVMode_UNKNOWN = 0,

		PANORAMA = 1, // Direct convert Azimuth to U and Elevation to V.
		PANORAMA_EQUIRECTANGULAR = 2, // Equirectangular projection.
	};

#define CoodMode_NUM_BITS 16
	enum CoodMode : Flag
	{
		CoodMode_UNKNOWN = 0,

		XYZ = 1, // Cartesian coordinate system: XYZ(X, Y, Z).
		RAE = 2, // Spherical coordinate system: RAE(Radius, Azimuth, Elevation).
	};

#define ElevationMode_NUM_BITS 2
	enum ElevationMode : Flag
	{
		ElevationMode_UNKNOWN = 0,

		NP = 1, // Elevation is the angle between vector and north pole which is cross of RAE(1, 0, 0) and RAE(1, 0, M_PI/2).
		SP = 2, // Elevation is the angle between vector and south pole which south pole is negative north pole.
		PE = 3, // Elevation is the angle between vector and plane of the ecliptic, and positive side is toward north pole.
	};

	//CoordSys format - 16 bit:
	//	X: means 0 or 1, but must at least exist a X is one
	//	*: means still usable space
	//
	//	5432109876543210
	//	_____XXXXXXXXXXX: _ bits encode CoodMode
	//	XXXXX___________: _ bits encode detailed info
	//
	//	5432109876543210
	//	00001XXXXXXXXXXX: XYZ
	//	00001**___XXXXXX: XYZ - _ bits encode I unit vector is which axis.
	//	00001**XXX___XXX: XYZ - _ bits encode J unit vector is which axis.
	//	00001**XXXXXX___: XYZ - _ bits encode K unit vector is which axis.
	//
	//	5432109876543210
	//	00002XXXXXXXXXXX: RAE
	//	00002***__XXXXXX: RAE - _ bits encode ElevationMode
	//	00002***XX___XXX: RAE - _ bits encode RAE(1, 0, 0) is which axis.
	//	00002***XXXXX___: RAE - _ bits encode RAE(1, 0, M_PI/2) is which axis.

#define CoodSys_NUM_BITS			16
#define CoodSys_CoordInfo_NUM_BITS	11
	enum CoordSys : Flag
	{
		CoordSys_UNKNOWN = 0,

		// XYZ
		XYZ_PX_PY_PZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::PX << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PZ,
		XYZ_PX_PY_NZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::PX << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NZ,
		XYZ_PX_NY_PZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::PX << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PZ,
		XYZ_PX_NY_NZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::PX << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NZ,
		XYZ_NX_PY_PZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::NX << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PZ,
		XYZ_NX_PY_NZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::NX << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NZ,
		XYZ_NX_NY_PZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::NX << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PZ,
		XYZ_NX_NY_NZ = (CoodMode::XYZ << CoodSys_CoordInfo_NUM_BITS) | (Axis::NX << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NZ,

		// RAE
		RAE_NP_PX_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::PY,
		RAE_NP_PX_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::PZ,
		RAE_NP_PX_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::NY,
		RAE_NP_PX_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::NZ,
		RAE_NP_PY_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PX,
		RAE_NP_PY_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PZ,
		RAE_NP_PY_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NX,
		RAE_NP_PY_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NZ,
		RAE_NP_PZ_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::PX,
		RAE_NP_PZ_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::PY,
		RAE_NP_PZ_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::NX,
		RAE_NP_PZ_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::NY,
		RAE_NP_NX_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::PY,
		RAE_NP_NX_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::PZ,
		RAE_NP_NX_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::NY,
		RAE_NP_NX_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::NZ,
		RAE_NP_NY_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PX,
		RAE_NP_NY_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PZ,
		RAE_NP_NY_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NX,
		RAE_NP_NY_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NZ,
		RAE_NP_NZ_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::PX,
		RAE_NP_NZ_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::PY,
		RAE_NP_NZ_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::NX,
		RAE_NP_NZ_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::NP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::NY,

		RAE_SP_PX_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::PY,
		RAE_SP_PX_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::PZ,
		RAE_SP_PX_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::NY,
		RAE_SP_PX_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::NZ,
		RAE_SP_PY_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PX,
		RAE_SP_PY_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PZ,
		RAE_SP_PY_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NX,
		RAE_SP_PY_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NZ,
		RAE_SP_PZ_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::PX,
		RAE_SP_PZ_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::PY,
		RAE_SP_PZ_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::NX,
		RAE_SP_PZ_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::NY,
		RAE_SP_NX_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::PY,
		RAE_SP_NX_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::PZ,
		RAE_SP_NX_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::NY,
		RAE_SP_NX_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::NZ,
		RAE_SP_NY_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PX,
		RAE_SP_NY_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PZ,
		RAE_SP_NY_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NX,
		RAE_SP_NY_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NZ,
		RAE_SP_NZ_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::PX,
		RAE_SP_NZ_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::PY,
		RAE_SP_NZ_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::NX,
		RAE_SP_NZ_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::SP << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::NY,

		RAE_PE_PX_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::PY,
		RAE_PE_PX_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::PZ,
		RAE_PE_PX_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::NY,
		RAE_PE_PX_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PX << AXIS_NUM_BITS) | Axis::NZ,
		RAE_PE_PY_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PX,
		RAE_PE_PY_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::PZ,
		RAE_PE_PY_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NX,
		RAE_PE_PY_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PY << AXIS_NUM_BITS) | Axis::NZ,
		RAE_PE_PZ_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::PX,
		RAE_PE_PZ_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::PY,
		RAE_PE_PZ_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::NX,
		RAE_PE_PZ_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::PZ << AXIS_NUM_BITS) | Axis::NY,
		RAE_PE_NX_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::PY,
		RAE_PE_NX_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::PZ,
		RAE_PE_NX_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::NY,
		RAE_PE_NX_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NX << AXIS_NUM_BITS) | Axis::NZ,
		RAE_PE_NY_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PX,
		RAE_PE_NY_PZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::PZ,
		RAE_PE_NY_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NX,
		RAE_PE_NY_NZ = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NY << AXIS_NUM_BITS) | Axis::NZ,
		RAE_PE_NZ_PX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::PX,
		RAE_PE_NZ_PY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::PY,
		RAE_PE_NZ_NX = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::NX,
		RAE_PE_NZ_NY = (CoodMode::RAE << CoodSys_CoordInfo_NUM_BITS) | (ElevationMode::PE << AXIS_2_NUM_BITS) | (Axis::NZ << AXIS_NUM_BITS) | Axis::NY,

		// 
	};

	//Mapping format:
	//	X: means 0 or 1, but must at least exist a X is one
	//	*: means still usable space
	//
	//	10987654321098765432109876543210
	//	________________XXXXXXXXXXXXXXXX: _ bits encode UVMode
	//	XXXXXXXXXXXXXXXX________________: _ bits encode CoordSys
	//

#define Mapping_NUM_BITS 32
	using Mapping = Flag;
	
	//
	inline Mapping ToMapping(UVMode uvMode, CoordSys coordSys);

	template<CoordSys outType, CoordSys inType>
	inline Eigen::Vector3d CoodConvert(const Eigen::Vector3d& coord);

	inline Eigen::Vector2d ToUV(Mapping mapping, const Eigen::Vector3d& coord);
}

#include "Coordinate.hpp"