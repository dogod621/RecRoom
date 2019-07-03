#pragma once

#include "Coordinate.h"

// Macros
//											  10987654321098765432109876543210
#define CoodSys_CoodMode_MASK				0b00000000000000001111100000000000
#define CoodSys_CoordInfo_MASK				0b00000000000000000000011111111111
#define CoodSys_XYZ_I_MASK					0b00000000000000000000000111000000
#define CoodSys_XYZ_J_MASK					0b00000000000000000000000000111000
#define CoodSys_XYZ_K_MASK					0b00000000000000000000000000000111
#define CoodSys_RAE_ElevationMode_MASK		0b00000000000000000000000011000000
#define CoodSys_RAE_1_0_0_MASK				0b00000000000000000000000000111000
#define CoodSys_RAE_1_0_90_MASK				0b00000000000000000000000000000111
#define CoodSys_CoodMode(v)					(RecRoom::CoodMode		)((v&CoodSys_CoodMode_MASK			) >> CoodSys_CoordInfo_NUM_BITS)
#define CoodSys_CoordInfo(v)				(RecRoom::Flag			) (v&CoodSys_CoordInfo_MASK)
#define CoodSys_XYZ_I(v)					(RecRoom::Axis			)((v&CoodSys_XYZ_I_MASK				) >> AXIS_2_NUM_BITS)
#define CoodSys_XYZ_J(v)					(RecRoom::Axis			)((v&CoodSys_XYZ_J_MASK				) >> AXIS_NUM_BITS)
#define CoodSys_XYZ_K(v)					(RecRoom::Axis			) (v&CoodSys_XYZ_K_MASK				)
#define CoodSys_RAE_ElevationMode(v)		(RecRoom::ElevationMode	)((v&CoodSys_RAE_ElevationMode_MASK	) >> AXIS_2_NUM_BITS)
#define CoodSys_RAE_1_0_0(v)				(RecRoom::Axis			)((v&CoodSys_RAE_1_0_0_MASK			) >> AXIS_NUM_BITS)
#define CoodSys_RAE_1_0_90(v)				(RecRoom::Axis			) (v&CoodSys_RAE_1_0_90_MASK		)

//											  10987654321098765432109876543210
#define Mapping_UVMode_MASK					0b11111111111111110000000000000000
#define Mapping_CoodSys_MASK				0b00000000000000001111111111111111
#define Mapping_UVMode(v)					(RecRoom::UVMode	)((v&Mapping_UVMode_MASK	) >> CoodSys_NUM_BITS)
#define Mapping_CoodSys(v)					(RecRoom::CoordSys	) (v&Mapping_CoodSys_MASK)
#define Mapping_CoodMode(v)					CoodSys_CoodMode(v)
#define Mapping_CoordInfo(v)				CoodSys_CoordInfo(v)
#define Mapping_XYZ_I(v)					CoodSys_XYZ_I(v)
#define Mapping_XYZ_J(v)					CoodSys_XYZ_J(v)
#define Mapping_XYZ_K(v)					CoodSys_XYZ_K(v)
#define Mapping_RAE_ElevationMode(v)		CoodSys_RAE_ElevationMode(v)
#define Mapping_RAE_1_0_0(v)				CoodSys_RAE_1_0_0(v)
#define Mapping_RAE_1_0_90(v)				CoodSys_RAE_1_0_90(v)

namespace RecRoom
{
	inline Mapping ToMapping(UVMode uvMode, CoordSys coordSys)
	{
		return (Mapping)((uvMode << CoodSys_NUM_BITS) | coordSys);
	}

	template<> 
	inline Eigen::Vector3d Convert<Eigen::Vector3d, Axis>(const Axis& v)
	{
		switch (v)
		{
		case Axis::PX: return Eigen::Vector3d(1.0, 0.0, 0.0); break;
		case Axis::PY: return Eigen::Vector3d(0.0, 1.0, 0.0); break;
		case Axis::PZ: return Eigen::Vector3d(0.0, 0.0, 1.0); break;
		case Axis::NX: return Eigen::Vector3d(-1.0, 0.0, 0.0); break;
		case Axis::NY: return Eigen::Vector3d(0.0, -1.0, 0.0); break;
		case Axis::NZ: return Eigen::Vector3d(0.0, 0.0, -1.0); break;
		default: THROW_EXCEPTION("Axis is not support"); break;
		}
	}

	template<CoordSys type>
	inline Eigen::Vector3d XYZ_To_Cartesian(const Eigen::Vector3d& coord)
	{
		if (type == CoordSys::XYZ_PX_PY_PZ)
			return coord;
		else
		{
			return \
				coord.x()*Convert<Eigen::Vector3d, Axis>(CoodSys_XYZ_I(type)) + \
				coord.y()*Convert<Eigen::Vector3d, Axis>(CoodSys_XYZ_J(type)) + \
				coord.z()*Convert<Eigen::Vector3d, Axis>(CoodSys_XYZ_K(type));
		}
	}

	template<CoordSys type>
	inline Eigen::Vector3d Cartesian_To_XYZ(const Eigen::Vector3d& coord)
	{
		if (type == CoordSys::XYZ_PX_PY_PZ)
			return coord;
		else
		{
			return Eigen::Vector3d(
				Convert<Eigen::Vector3d, Axis>(CoodSys_XYZ_I(type)).dot(coord),
				Convert<Eigen::Vector3d, Axis>(CoodSys_XYZ_J(type)).dot(coord),
				Convert<Eigen::Vector3d, Axis>(CoodSys_XYZ_K(type)).dot(coord));
		}
	}

	template<CoordSys type>
	inline Eigen::Vector3d RAE_To_Cartesian(const Eigen::Vector3d& coord)
	{
		double r = coord.x();
		double s_e = std::sin(coord.z());
		double s_a = std::sin(coord.y());
		double c_e = std::cos(coord.z());
		double c_a = std::cos(coord.y());
		Eigen::Vector3d v_1_0_0 = Convert<Eigen::Vector3d, Axis>(CoodSys_RAE_1_0_0(type));
		Eigen::Vector3d v_1_0_90 = Convert<Eigen::Vector3d, Axis>(CoodSys_RAE_1_0_90(type));
		Eigen::Vector3d np = v_1_0_0.cross(v_1_0_90);

		//
		double len_np;
		double len_pe;
		switch (CoodSys_RAE_ElevationMode(type))
		{
		case ElevationMode::NP:
			len_np = r * c_e;
			len_pe = r * s_e;
			break;
		case ElevationMode::SP:
			len_np = r * (-c_e);
			len_pe = r * s_e;
			break;
		case ElevationMode::PE:
			len_np = r * s_e;
			len_pe = r * c_e;
			break;
		default:
			THROW_EXCEPTION("ElevationMode is not support");
			break;
		}
		return v_1_0_0 * len_pe * c_a + v_1_0_90 * len_pe * s_a + np * len_np;
	}

	template<CoordSys type>
	inline Eigen::Vector3d Cartesian_To_RAE(const Eigen::Vector3d& coord)
	{
		Eigen::Vector3d rae;
		rae.x() = coord.norm();
		Eigen::Vector3d xyz = coord / rae.x();
		Eigen::Vector3d v_1_0_0 = Convert<Eigen::Vector3d, Axis>(CoodSys_RAE_1_0_0(type));
		Eigen::Vector3d v_1_0_90 = Convert<Eigen::Vector3d, Axis>(CoodSys_RAE_1_0_90(type));
		Eigen::Vector3d np = v_1_0_0.cross(v_1_0_90);

		double len_np = xyz.dot(np);
		switch (CoodSys_RAE_ElevationMode(type))
		{
		case ElevationMode::NP:
			rae.z() = std::acos(len_np);
			break;
		case ElevationMode::SP:
			rae.z() = M_PI - std::acos(len_np);
			break;
		case ElevationMode::PE:
			rae.z() = std::asin(len_np);
			break;
		default:
			THROW_EXCEPTION("outType ElevationMode is not support");
			break;
		}

		Eigen::Vector3d v_ep = xyz - len_np * np;
		rae.y() = std::atan2(v_ep.dot(v_1_0_90), v_ep.dot(v_1_0_0));
		return rae;
	}

	template<CoordSys outType, CoordSys inType>
	inline Eigen::Vector3d CoodConvert(const Eigen::Vector3d& coord)
	{
		switch (CoodSys_CoodMode(inType))
		{
		case CoodMode::XYZ:
			switch (CoodSys_CoodMode(outType))
			{
			case CoodMode::XYZ: return Cartesian_To_XYZ<outType>(XYZ_To_Cartesian<inType>(coord)); break;
			case CoodMode::RAE: return Cartesian_To_RAE<outType>(XYZ_To_Cartesian<inType>(coord)); break;
			default: THROW_EXCEPTION("outType is not support"); break;
			}
			break;

		case CoodMode::RAE:
			switch (CoodSys_CoodMode(outType))
			{
			case CoodMode::XYZ: return Cartesian_To_XYZ<outType>(RAE_To_Cartesian<inType>(coord)); break;
			case CoodMode::RAE: return Cartesian_To_RAE<outType>(RAE_To_Cartesian<inType>(coord)); break;
			default: THROW_EXCEPTION("outType is not support"); break;
			}
			break;

		default:
			THROW_EXCEPTION("inType is not support");
			break;
		}
	}

	inline Eigen::Vector2d XYZ_To_UV(Mapping mapping, const Eigen::Vector3d& xyz)
	{
		Eigen::Vector2d uv;

		switch (Mapping_UVMode(mapping))
		{
		case UVMode::HEMISPHERE:
		{
			Eigen::Vector3d xyz2 = xyz / xyz.norm();
			uv.x() = ( xyz2.x() + 1.0 ) * 0.5;
			uv.y() = ( xyz2.y() + 1.0 ) * 0.5;
		}
		break;

		default:
			THROW_EXCEPTION("UVMode is not support");
			break;
		}

		uv.x() = std::min(std::max(0.0, uv.x()), 1.0);
		uv.y() = std::min(std::max(0.0, uv.y()), 1.0);
		return uv;
	}

	inline Eigen::Vector2d RAE_To_UV(Mapping mapping, const Eigen::Vector3d& rae)
	{
		Eigen::Vector2d uv;

		uv.x() = 0.5 * (1.0 - rae.y() * M_1_PI);

		switch (Mapping_UVMode(mapping))
		{
		case UVMode::PANORAMA:
			switch (Mapping_RAE_ElevationMode(mapping))
			{
			case ElevationMode::PE: uv.y() = 0.5 * (1.0 + rae.z() / M_PI_2); break;
			case ElevationMode::NP: uv.y() = (1.0 - rae.z() * M_1_PI); break;
			case ElevationMode::SP: uv.y() = rae.z() * M_1_PI; break;
			default: THROW_EXCEPTION("ElevationMode is not support"); break;
			}
			break;

		case UVMode::PANORAMA_EQUIRECTANGULAR:
			switch (Mapping_RAE_ElevationMode(mapping))
			{
			case ElevationMode::PE: uv.y() = 0.5 * (1.0 + std::sin(rae.z())); break;
			case ElevationMode::NP: uv.y() = std::cos(rae.z()); break;
			case ElevationMode::SP: uv.y() = 1.0 - std::cos(rae.z()); break;
			default: THROW_EXCEPTION("ElevationMode is not support"); break;
			}
			break;

		default:
			THROW_EXCEPTION("UVMode is not support");
			break;
		}

		uv.x() = std::min(std::max(0.0, uv.x()), 1.0);
		uv.y() = std::min(std::max(0.0, uv.y()), 1.0);
		return uv;
	}

	inline Eigen::Vector2d ToUV(Mapping mapping, const Eigen::Vector3d& coord)
	{
		switch (Mapping_CoodMode(mapping))
		{
		case CoodMode::XYZ: return XYZ_To_UV(mapping, coord); break;
		case CoodMode::RAE: return RAE_To_UV(mapping, coord); break;
		default: THROW_EXCEPTION("CoordSys is not support"); break;
		}
	}

	template<>
	inline Axis Convert<Axis, std::string>(const std::string& v)
	{
		if (v == "PX") return Axis::PX;
		else if (v == "PY") return Axis::PY;
		else if (v == "PZ") return Axis::PZ;
		else if (v == "NX") return Axis::NX;
		else if (v == "NY") return Axis::NY;
		else if (v == "NZ") return Axis::NZ;
		else return Axis::Axis_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, Axis>(const Axis& v)
	{
		switch (v)
		{
		case Axis::PX: return std::string("PX"); break;
		case Axis::PY: return std::string("PY"); break;
		case Axis::PZ: return std::string("PZ"); break;
		case Axis::NX: return std::string("NX"); break;
		case Axis::NY: return std::string("NY"); break;
		case Axis::NZ: return std::string("NZ"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline UVMode Convert<UVMode, std::string>(const std::string& v)
	{
		if (v == "PANORAMA") return UVMode::PANORAMA;
		else if (v == "PANORAMA_EQUIRECTANGULAR") return UVMode::PANORAMA_EQUIRECTANGULAR;
		else if (v == "HEMISPHERE") return UVMode::HEMISPHERE;
		else return UVMode::UVMode_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, UVMode>(const UVMode& v)
	{
		switch (v)
		{
		case UVMode::PANORAMA: return std::string("PANORAMA"); break;
		case UVMode::PANORAMA_EQUIRECTANGULAR: return std::string("PANORAMA_EQUIRECTANGULAR"); break;
		case UVMode::HEMISPHERE: return std::string("HEMISPHERE"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline CoodMode Convert<CoodMode, std::string>(const std::string& v)
	{
		if (v == "XYZ") return CoodMode::XYZ;
		else if (v == "RAE") return CoodMode::RAE;
		else return CoodMode::CoodMode_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, CoodMode>(const CoodMode& v)
	{
		switch (v)
		{
		case CoodMode::XYZ: return std::string("XYZ"); break;
		case CoodMode::RAE: return std::string("RAE"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline ElevationMode Convert<ElevationMode, std::string>(const std::string& v)
	{
		if (v == "NP") return ElevationMode::NP;
		else if (v == "SP") return ElevationMode::SP;
		else if (v == "PE") return ElevationMode::PE;
		else return ElevationMode::ElevationMode_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, ElevationMode>(const ElevationMode& v)
	{
		switch (v)
		{
		case ElevationMode::NP: return std::string("NP"); break;
		case ElevationMode::SP: return std::string("SP"); break;
		case ElevationMode::PE: return std::string("PE"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline CoordSys Convert<CoordSys, std::string>(const std::string& v)
	{
		if (v == "XYZ_PX_PY_PZ") return CoordSys::XYZ_PX_PY_PZ;
		else if (v == "XYZ_PX_PY_NZ") return CoordSys::XYZ_PX_PY_NZ;
		else if (v == "XYZ_PX_NY_PZ") return CoordSys::XYZ_PX_NY_PZ;
		else if (v == "XYZ_PX_NY_NZ") return CoordSys::XYZ_PX_NY_NZ;
		else if (v == "XYZ_NX_PY_PZ") return CoordSys::XYZ_NX_PY_PZ;
		else if (v == "XYZ_NX_PY_NZ") return CoordSys::XYZ_NX_PY_NZ;
		else if (v == "XYZ_NX_NY_PZ") return CoordSys::XYZ_NX_NY_PZ;
		else if (v == "XYZ_NX_NY_NZ") return CoordSys::XYZ_NX_NY_NZ;

		else if (v == "RAE_NP_PX_PY") return CoordSys::RAE_NP_PX_PY;
		else if (v == "RAE_NP_PX_PZ") return CoordSys::RAE_NP_PX_PZ;
		else if (v == "RAE_NP_PX_NY") return CoordSys::RAE_NP_PX_NY;
		else if (v == "RAE_NP_PX_NZ") return CoordSys::RAE_NP_PX_NZ;
		else if (v == "RAE_NP_PY_PX") return CoordSys::RAE_NP_PY_PX;
		else if (v == "RAE_NP_PY_PZ") return CoordSys::RAE_NP_PY_PZ;
		else if (v == "RAE_NP_PY_NX") return CoordSys::RAE_NP_PY_NX;
		else if (v == "RAE_NP_PY_NZ") return CoordSys::RAE_NP_PY_NZ;
		else if (v == "RAE_NP_PZ_PX") return CoordSys::RAE_NP_PZ_PX;
		else if (v == "RAE_NP_PZ_PY") return CoordSys::RAE_NP_PZ_PY;
		else if (v == "RAE_NP_PZ_NX") return CoordSys::RAE_NP_PZ_NX;
		else if (v == "RAE_NP_PZ_NY") return CoordSys::RAE_NP_PZ_NY;
		else if (v == "RAE_NP_NX_PY") return CoordSys::RAE_NP_NX_PY;
		else if (v == "RAE_NP_NX_PZ") return CoordSys::RAE_NP_NX_PZ;
		else if (v == "RAE_NP_NX_NY") return CoordSys::RAE_NP_NX_NY;
		else if (v == "RAE_NP_NX_NZ") return CoordSys::RAE_NP_NX_NZ;
		else if (v == "RAE_NP_NY_PX") return CoordSys::RAE_NP_NY_PX;
		else if (v == "RAE_NP_NY_PZ") return CoordSys::RAE_NP_NY_PZ;
		else if (v == "RAE_NP_NY_NX") return CoordSys::RAE_NP_NY_NX;
		else if (v == "RAE_NP_NY_NZ") return CoordSys::RAE_NP_NY_NZ;
		else if (v == "RAE_NP_NZ_PX") return CoordSys::RAE_NP_NZ_PX;
		else if (v == "RAE_NP_NZ_PY") return CoordSys::RAE_NP_NZ_PY;
		else if (v == "RAE_NP_NZ_NX") return CoordSys::RAE_NP_NZ_NX;
		else if (v == "RAE_NP_NZ_NY") return CoordSys::RAE_NP_NZ_NY;

		else if (v == "RAE_SP_PX_PY") return CoordSys::RAE_SP_PX_PY;
		else if (v == "RAE_SP_PX_PZ") return CoordSys::RAE_SP_PX_PZ;
		else if (v == "RAE_SP_PX_NY") return CoordSys::RAE_SP_PX_NY;
		else if (v == "RAE_SP_PX_NZ") return CoordSys::RAE_SP_PX_NZ;
		else if (v == "RAE_SP_PY_PX") return CoordSys::RAE_SP_PY_PX;
		else if (v == "RAE_SP_PY_PZ") return CoordSys::RAE_SP_PY_PZ;
		else if (v == "RAE_SP_PY_NX") return CoordSys::RAE_SP_PY_NX;
		else if (v == "RAE_SP_PY_NZ") return CoordSys::RAE_SP_PY_NZ;
		else if (v == "RAE_SP_PZ_PX") return CoordSys::RAE_SP_PZ_PX;
		else if (v == "RAE_SP_PZ_PY") return CoordSys::RAE_SP_PZ_PY;
		else if (v == "RAE_SP_PZ_NX") return CoordSys::RAE_SP_PZ_NX;
		else if (v == "RAE_SP_PZ_NY") return CoordSys::RAE_SP_PZ_NY;
		else if (v == "RAE_SP_NX_PY") return CoordSys::RAE_SP_NX_PY;
		else if (v == "RAE_SP_NX_PZ") return CoordSys::RAE_SP_NX_PZ;
		else if (v == "RAE_SP_NX_NY") return CoordSys::RAE_SP_NX_NY;
		else if (v == "RAE_SP_NX_NZ") return CoordSys::RAE_SP_NX_NZ;
		else if (v == "RAE_SP_NY_PX") return CoordSys::RAE_SP_NY_PX;
		else if (v == "RAE_SP_NY_PZ") return CoordSys::RAE_SP_NY_PZ;
		else if (v == "RAE_SP_NY_NX") return CoordSys::RAE_SP_NY_NX;
		else if (v == "RAE_SP_NY_NZ") return CoordSys::RAE_SP_NY_NZ;
		else if (v == "RAE_SP_NZ_PX") return CoordSys::RAE_SP_NZ_PX;
		else if (v == "RAE_SP_NZ_PY") return CoordSys::RAE_SP_NZ_PY;
		else if (v == "RAE_SP_NZ_NX") return CoordSys::RAE_SP_NZ_NX;
		else if (v == "RAE_SP_NZ_NY") return CoordSys::RAE_SP_NZ_NY;

		else if (v == "RAE_PE_PX_PY") return CoordSys::RAE_PE_PX_PY;
		else if (v == "RAE_PE_PX_PZ") return CoordSys::RAE_PE_PX_PZ;
		else if (v == "RAE_PE_PX_NY") return CoordSys::RAE_PE_PX_NY;
		else if (v == "RAE_PE_PX_NZ") return CoordSys::RAE_PE_PX_NZ;
		else if (v == "RAE_PE_PY_PX") return CoordSys::RAE_PE_PY_PX;
		else if (v == "RAE_PE_PY_PZ") return CoordSys::RAE_PE_PY_PZ;
		else if (v == "RAE_PE_PY_NX") return CoordSys::RAE_PE_PY_NX;
		else if (v == "RAE_PE_PY_NZ") return CoordSys::RAE_PE_PY_NZ;
		else if (v == "RAE_PE_PZ_PX") return CoordSys::RAE_PE_PZ_PX;
		else if (v == "RAE_PE_PZ_PY") return CoordSys::RAE_PE_PZ_PY;
		else if (v == "RAE_PE_PZ_NX") return CoordSys::RAE_PE_PZ_NX;
		else if (v == "RAE_PE_PZ_NY") return CoordSys::RAE_PE_PZ_NY;
		else if (v == "RAE_PE_NX_PY") return CoordSys::RAE_PE_NX_PY;
		else if (v == "RAE_PE_NX_PZ") return CoordSys::RAE_PE_NX_PZ;
		else if (v == "RAE_PE_NX_NY") return CoordSys::RAE_PE_NX_NY;
		else if (v == "RAE_PE_NX_NZ") return CoordSys::RAE_PE_NX_NZ;
		else if (v == "RAE_PE_NY_PX") return CoordSys::RAE_PE_NY_PX;
		else if (v == "RAE_PE_NY_PZ") return CoordSys::RAE_PE_NY_PZ;
		else if (v == "RAE_PE_NY_NX") return CoordSys::RAE_PE_NY_NX;
		else if (v == "RAE_PE_NY_NZ") return CoordSys::RAE_PE_NY_NZ;
		else if (v == "RAE_PE_NZ_PX") return CoordSys::RAE_PE_NZ_PX;
		else if (v == "RAE_PE_NZ_PY") return CoordSys::RAE_PE_NZ_PY;
		else if (v == "RAE_PE_NZ_NX") return CoordSys::RAE_PE_NZ_NX;
		else if (v == "RAE_PE_NZ_NY") return CoordSys::RAE_PE_NZ_NY;

		else return CoordSys::CoordSys_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, CoordSys>(const CoordSys& v)
	{
		switch (v)
		{
		case CoordSys::XYZ_PX_PY_PZ: return std::string("XYZ_PX_PY_PZ"); break;
		case CoordSys::XYZ_PX_PY_NZ: return std::string("XYZ_PX_PY_NZ"); break;
		case CoordSys::XYZ_PX_NY_PZ: return std::string("XYZ_PX_NY_PZ"); break;
		case CoordSys::XYZ_PX_NY_NZ: return std::string("XYZ_PX_NY_NZ"); break;
		case CoordSys::XYZ_NX_PY_PZ: return std::string("XYZ_NX_PY_PZ"); break;
		case CoordSys::XYZ_NX_PY_NZ: return std::string("XYZ_NX_PY_NZ"); break;
		case CoordSys::XYZ_NX_NY_PZ: return std::string("XYZ_NX_NY_PZ"); break;
		case CoordSys::XYZ_NX_NY_NZ: return std::string("XYZ_NX_NY_NZ"); break;

		case CoordSys::RAE_NP_PX_PY: return std::string("RAE_NP_PX_PY"); break;
		case CoordSys::RAE_NP_PX_PZ: return std::string("RAE_NP_PX_PZ"); break;
		case CoordSys::RAE_NP_PX_NY: return std::string("RAE_NP_PX_NY"); break;
		case CoordSys::RAE_NP_PX_NZ: return std::string("RAE_NP_PX_NZ"); break;
		case CoordSys::RAE_NP_PY_PX: return std::string("RAE_NP_PY_PX"); break;
		case CoordSys::RAE_NP_PY_PZ: return std::string("RAE_NP_PY_PZ"); break;
		case CoordSys::RAE_NP_PY_NX: return std::string("RAE_NP_PY_NX"); break;
		case CoordSys::RAE_NP_PY_NZ: return std::string("RAE_NP_PY_NZ"); break;
		case CoordSys::RAE_NP_PZ_PX: return std::string("RAE_NP_PZ_PX"); break;
		case CoordSys::RAE_NP_PZ_PY: return std::string("RAE_NP_PZ_PY"); break;
		case CoordSys::RAE_NP_PZ_NX: return std::string("RAE_NP_PZ_NX"); break;
		case CoordSys::RAE_NP_PZ_NY: return std::string("RAE_NP_PZ_NY"); break;
		case CoordSys::RAE_NP_NX_PY: return std::string("RAE_NP_NX_PY"); break;
		case CoordSys::RAE_NP_NX_PZ: return std::string("RAE_NP_NX_PZ"); break;
		case CoordSys::RAE_NP_NX_NY: return std::string("RAE_NP_NX_NY"); break;
		case CoordSys::RAE_NP_NX_NZ: return std::string("RAE_NP_NX_NZ"); break;
		case CoordSys::RAE_NP_NY_PX: return std::string("RAE_NP_NY_PX"); break;
		case CoordSys::RAE_NP_NY_PZ: return std::string("RAE_NP_NY_PZ"); break;
		case CoordSys::RAE_NP_NY_NX: return std::string("RAE_NP_NY_NX"); break;
		case CoordSys::RAE_NP_NY_NZ: return std::string("RAE_NP_NY_NZ"); break;
		case CoordSys::RAE_NP_NZ_PX: return std::string("RAE_NP_NZ_PX"); break;
		case CoordSys::RAE_NP_NZ_PY: return std::string("RAE_NP_NZ_PY"); break;
		case CoordSys::RAE_NP_NZ_NX: return std::string("RAE_NP_NZ_NX"); break;
		case CoordSys::RAE_NP_NZ_NY: return std::string("RAE_NP_NZ_NY"); break;

		case CoordSys::RAE_SP_PX_PY: return std::string("RAE_SP_PX_PY"); break;
		case CoordSys::RAE_SP_PX_PZ: return std::string("RAE_SP_PX_PZ"); break;
		case CoordSys::RAE_SP_PX_NY: return std::string("RAE_SP_PX_NY"); break;
		case CoordSys::RAE_SP_PX_NZ: return std::string("RAE_SP_PX_NZ"); break;
		case CoordSys::RAE_SP_PY_PX: return std::string("RAE_SP_PY_PX"); break;
		case CoordSys::RAE_SP_PY_PZ: return std::string("RAE_SP_PY_PZ"); break;
		case CoordSys::RAE_SP_PY_NX: return std::string("RAE_SP_PY_NX"); break;
		case CoordSys::RAE_SP_PY_NZ: return std::string("RAE_SP_PY_NZ"); break;
		case CoordSys::RAE_SP_PZ_PX: return std::string("RAE_SP_PZ_PX"); break;
		case CoordSys::RAE_SP_PZ_PY: return std::string("RAE_SP_PZ_PY"); break;
		case CoordSys::RAE_SP_PZ_NX: return std::string("RAE_SP_PZ_NX"); break;
		case CoordSys::RAE_SP_PZ_NY: return std::string("RAE_SP_PZ_NY"); break;
		case CoordSys::RAE_SP_NX_PY: return std::string("RAE_SP_NX_PY"); break;
		case CoordSys::RAE_SP_NX_PZ: return std::string("RAE_SP_NX_PZ"); break;
		case CoordSys::RAE_SP_NX_NY: return std::string("RAE_SP_NX_NY"); break;
		case CoordSys::RAE_SP_NX_NZ: return std::string("RAE_SP_NX_NZ"); break;
		case CoordSys::RAE_SP_NY_PX: return std::string("RAE_SP_NY_PX"); break;
		case CoordSys::RAE_SP_NY_PZ: return std::string("RAE_SP_NY_PZ"); break;
		case CoordSys::RAE_SP_NY_NX: return std::string("RAE_SP_NY_NX"); break;
		case CoordSys::RAE_SP_NY_NZ: return std::string("RAE_SP_NY_NZ"); break;
		case CoordSys::RAE_SP_NZ_PX: return std::string("RAE_SP_NZ_PX"); break;
		case CoordSys::RAE_SP_NZ_PY: return std::string("RAE_SP_NZ_PY"); break;
		case CoordSys::RAE_SP_NZ_NX: return std::string("RAE_SP_NZ_NX"); break;
		case CoordSys::RAE_SP_NZ_NY: return std::string("RAE_SP_NZ_NY"); break;

		case CoordSys::RAE_PE_PX_PY: return std::string("RAE_PE_PX_PY"); break;
		case CoordSys::RAE_PE_PX_PZ: return std::string("RAE_PE_PX_PZ"); break;
		case CoordSys::RAE_PE_PX_NY: return std::string("RAE_PE_PX_NY"); break;
		case CoordSys::RAE_PE_PX_NZ: return std::string("RAE_PE_PX_NZ"); break;
		case CoordSys::RAE_PE_PY_PX: return std::string("RAE_PE_PY_PX"); break;
		case CoordSys::RAE_PE_PY_PZ: return std::string("RAE_PE_PY_PZ"); break;
		case CoordSys::RAE_PE_PY_NX: return std::string("RAE_PE_PY_NX"); break;
		case CoordSys::RAE_PE_PY_NZ: return std::string("RAE_PE_PY_NZ"); break;
		case CoordSys::RAE_PE_PZ_PX: return std::string("RAE_PE_PZ_PX"); break;
		case CoordSys::RAE_PE_PZ_PY: return std::string("RAE_PE_PZ_PY"); break;
		case CoordSys::RAE_PE_PZ_NX: return std::string("RAE_PE_PZ_NX"); break;
		case CoordSys::RAE_PE_PZ_NY: return std::string("RAE_PE_PZ_NY"); break;
		case CoordSys::RAE_PE_NX_PY: return std::string("RAE_PE_NX_PY"); break;
		case CoordSys::RAE_PE_NX_PZ: return std::string("RAE_PE_NX_PZ"); break;
		case CoordSys::RAE_PE_NX_NY: return std::string("RAE_PE_NX_NY"); break;
		case CoordSys::RAE_PE_NX_NZ: return std::string("RAE_PE_NX_NZ"); break;
		case CoordSys::RAE_PE_NY_PX: return std::string("RAE_PE_NY_PX"); break;
		case CoordSys::RAE_PE_NY_PZ: return std::string("RAE_PE_NY_PZ"); break;
		case CoordSys::RAE_PE_NY_NX: return std::string("RAE_PE_NY_NX"); break;
		case CoordSys::RAE_PE_NY_NZ: return std::string("RAE_PE_NY_NZ"); break;
		case CoordSys::RAE_PE_NZ_PX: return std::string("RAE_PE_NZ_PX"); break;
		case CoordSys::RAE_PE_NZ_PY: return std::string("RAE_PE_NZ_PY"); break;
		case CoordSys::RAE_PE_NZ_NX: return std::string("RAE_PE_NZ_NX"); break;
		case CoordSys::RAE_PE_NZ_NY: return std::string("RAE_PE_NZ_NY"); break;

		default: return std::string("UNKNOWN"); break;
		}
	}
}