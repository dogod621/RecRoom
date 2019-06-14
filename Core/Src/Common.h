#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <string>
#include <exception>
#include <Eigen/Core>

namespace RecRoom
{
	class exception : public std::exception
	{
	public:
		exception(const std::string &msg, const char *file, int line) 
			: std::exception()
		{
			std::ostringstream o;
			o << file << ":" << line << "-" << msg;
			message = o.str();
		}
		~exception() {}
		const char *what() const { return message.c_str(); }

	protected:
		std::string message;
	};

#define THROW_EXCEPTION(message) throw RecRoom::exception(message, __FILE__, __LINE__);

	//
	using Flag = unsigned int;

	//
	enum Scanner : Flag
	{
		Scaner_UNKNOWN = 0,

		BLK360 = 1,
	};

	enum LinearSolver : Flag
	{
		LinearSolver_UNKNOWN = 0,

		EIGEN_QR = 1,
		EIGEN_SVD = 2,
		EIGEN_NE = 3,
	};

	// 0~7 = 3 bits
	enum Axis
	{
		Axis_UNKNOWN = 0,

		X = 1,
		Y = 2,
		Z = 3,

		NX = 4,
		NY = 5,
		NZ = 6,
	};

	// 0~3 = 2 bits
	enum _RAEElevationMode : Flag
	{
		_RAEElevationMode_UNKNOWN = 0,

		_N = 1, // Elevation is the angle between measure vector and north pole, north pole is cross of these two vectore on plane of the ecliptic: Azimuth at 0 degree, Azimuth at 90 degree
		_S = 2, // Elevation is the angle between measure vector and south pole
		_E = 3, // Elevation is the angle between measure vector and plane of the ecliptic, and positive side is toward north pole
	};

	// 3 + 3 = 6 bits
	enum _RAEAzimuthMode : Flag
	{
		_RAEAzimuthMode_UNKNOWN = 0,

		_X_Y = ((Axis::X << 3) | Axis::Y), // Azimuth is the angle between measure vector(project on X-Y plane) and X axis, and Y axis is 90 degree
		_X_Z = ((Axis::X << 3) | Axis::Z), // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and Z axis is 90 degree
		_X_NY = ((Axis::X << 3) | Axis::NY), // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and negtive Y axis is 90 degree
		_X_NZ = ((Axis::X << 3) | Axis::NZ), // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and negtive Z axis is 90 degree

		_Y_X = ((Axis::Y << 3) | Axis::X), // Azimuth is the angle between measure vector(project on Y-X plane) and Y axis, and X axis is 90 degree
		_Y_Z = ((Axis::Y << 3) | Axis::Z), // Azimuth is the angle between measure vector(project on Y-Z plane) and Y axis, and Z axis is 90 degree
		_Y_NX = ((Axis::Y << 3) | Axis::NX), // Azimuth is the angle between measure vector(project on Y-X plane) and Y axis, and negtive X axis is 90 degree
		_Y_NZ = ((Axis::Y << 3) | Axis::NZ), // Azimuth is the angle between measure vector(project on Y-Z plane) and Y axis, and negtive Z axis is 90 degree

		_Z_X = ((Axis::Z << 3) | Axis::X), // Azimuth is the angle between measure vector(project on Z-X plane) and Z axis, and X axis is 90 degree
		_Z_Y = ((Axis::Z << 3) | Axis::Y), // Azimuth is the angle between measure vector(project on Z-Y plane) and Z axis, and Y axis is 90 degree
		_Z_NX = ((Axis::Z << 3) | Axis::NX), // Azimuth is the angle between measure vector(project on Z-X plane) and Z axis, and negtive X axis is 90 degree
		_Z_NY = ((Axis::Z << 3) | Axis::NY), // Azimuth is the angle between measure vector(project on Z-Y plane) and Z axis, and negtive Y axis is 90 degree

		_NX_Y = ((Axis::NX << 3) | Axis::Y), // Azimuth is the angle between measure vector(project on X-Y plane) and negtive X axis, and Y axis is 90 degree
		_NX_Z = ((Axis::NX << 3) | Axis::Z), // Azimuth is the angle between measure vector(project on X-Z plane) and negtive X axis, and Z axis is 90 degree
		_NX_NY = ((Axis::NX << 3) | Axis::NY), // Azimuth is the angle between measure vector(project on X-Z plane) and negtive X axis, and negtive Y axis is 90 degree
		_NX_NZ = ((Axis::NX << 3) | Axis::NZ), // Azimuth is the angle between measure vector(project on X-Z plane) and negtive X axis, and negtive Z axis is 90 degree

		_NY_X = ((Axis::NY << 3) | Axis::X), // Azimuth is the angle between measure vector(project on Y-X plane) and negtive Y axis, and X axis is 90 degree
		_NY_Z = ((Axis::NY << 3) | Axis::Z), // Azimuth is the angle between measure vector(project on Y-Z plane) and negtive Y axis, and Z axis is 90 degree
		_NY_NX = ((Axis::NY << 3) | Axis::NX), // Azimuth is the angle between measure vector(project on Y-X plane) and negtive Y axis, and negtive X axis is 90 degree
		_NY_NZ = ((Axis::NY << 3) | Axis::NZ), // Azimuth is the angle between measure vector(project on Y-Z plane) and negtive Y axis, and negtive Z axis is 90 degree

		_NZ_X = ((Axis::NZ << 3) | Axis::X), // Azimuth is the angle between measure vector(project on Z-X plane) and negtive Z axis, and X axis is 90 degree
		_NZ_Y = ((Axis::NZ << 3) | Axis::Y), // Azimuth is the angle between measure vector(project on Z-Y plane) and negtive Z axis, and Y axis is 90 degree
		_NZ_NX = ((Axis::NZ << 3) | Axis::NX), // Azimuth is the angle between measure vector(project on Z-X plane) and negtive Z axis, and negtive X axis is 90 degree
		_NZ_NY = ((Axis::NZ << 3) | Axis::NY), // Azimuth is the angle between measure vector(project on Z-Y plane) and negtive Z axis, and negtive Y axis is 90 degree
	};

	// 0~65535 = 16 bit
	enum _UVMode
	{
		_UVMode_UNKNOWN = 0,

		_PANORAMA = 1, // Direct convert Azimuth to U and Elevation to V 
		_PANORAMA_EQUIRECTANGULAR = 2, // Equirectangular projection
	};

	enum _CoodMode : Flag
	{
		_CoodMode_UNKNOWN = 0,

		_XYZ = 1,
		_RAE = 2,
	};

	enum CoodSys : Flag
	{
		CoodSys_UNKNOWN = 0,

		// 0~1 bit is for XYZ
		XYZ = 1,

		// 1~9 bit is for RAE
		RAE_N_X_Y = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_X_Y) << 1,
		RAE_N_X_Z = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_X_Z) << 1,
		RAE_N_X_NY = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_X_NY) << 1,
		RAE_N_X_NZ = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_X_NZ) << 1,
		RAE_N_Y_X = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Y_X) << 1,
		RAE_N_Y_Z = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Y_Z) << 1,
		RAE_N_Y_NX = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Y_NX) << 1,
		RAE_N_Y_NZ = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Y_NZ) << 1,
		RAE_N_Z_X = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Z_X) << 1,
		RAE_N_Z_Y = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Z_Y) << 1,
		RAE_N_Z_NX = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Z_NX) << 1,
		RAE_N_Z_NY = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_Z_NY) << 1,
		RAE_N_NX_Y = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NX_Y) << 1,
		RAE_N_NX_Z = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NX_Z) << 1,
		RAE_N_NX_NY = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NX_NY) << 1,
		RAE_N_NX_NZ = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NX_NZ) << 1,
		RAE_N_NY_X = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NY_X) << 1,
		RAE_N_NY_Z = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NY_Z) << 1,
		RAE_N_NY_NX = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NY_NX) << 1,
		RAE_N_NY_NZ = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NY_NZ) << 1,
		RAE_N_NZ_X = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NZ_X) << 1,
		RAE_N_NZ_Y = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NZ_Y) << 1,
		RAE_N_NZ_NX = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NZ_NX) << 1,
		RAE_N_NZ_NY = ((_RAEElevationMode::_N << 6) | _RAEAzimuthMode::_NZ_NY) << 1,

		//
		RAE_S_X_Y = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_X_Y) << 1,
		RAE_S_X_Z = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_X_Z) << 1,
		RAE_S_X_NY = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_X_NY) << 1,
		RAE_S_X_NZ = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_X_NZ) << 1,
		RAE_S_Y_X = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Y_X) << 1,
		RAE_S_Y_Z = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Y_Z) << 1,
		RAE_S_Y_NX = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Y_NX) << 1,
		RAE_S_Y_NZ = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Y_NZ) << 1,
		RAE_S_Z_X = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Z_X) << 1,
		RAE_S_Z_Y = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Z_Y) << 1,
		RAE_S_Z_NX = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Z_NX) << 1,
		RAE_S_Z_NY = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_Z_NY) << 1,
		RAE_S_NX_Y = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NX_Y) << 1,
		RAE_S_NX_Z = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NX_Z) << 1,
		RAE_S_NX_NY = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NX_NY) << 1,
		RAE_S_NX_NZ = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NX_NZ) << 1,
		RAE_S_NY_X = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NY_X) << 1,
		RAE_S_NY_Z = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NY_Z) << 1,
		RAE_S_NY_NX = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NY_NX) << 1,
		RAE_S_NY_NZ = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NY_NZ) << 1,
		RAE_S_NZ_X = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NZ_X) << 1,
		RAE_S_NZ_Y = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NZ_Y) << 1,
		RAE_S_NZ_NX = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NZ_NX) << 1,
		RAE_S_NZ_NY = ((_RAEElevationMode::_S << 6) | _RAEAzimuthMode::_NZ_NY) << 1,

		//
		RAE_E_X_Y = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_X_Y) << 1,
		RAE_E_X_Z = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_X_Z) << 1,
		RAE_E_X_NY = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_X_NY) << 1,
		RAE_E_X_NZ = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_X_NZ) << 1,
		RAE_E_Y_X = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Y_X) << 1,
		RAE_E_Y_Z = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Y_Z) << 1,
		RAE_E_Y_NX = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Y_NX) << 1,
		RAE_E_Y_NZ = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Y_NZ) << 1,
		RAE_E_Z_X = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Z_X) << 1,
		RAE_E_Z_Y = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Z_Y) << 1,
		RAE_E_Z_NX = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Z_NX) << 1,
		RAE_E_Z_NY = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_Z_NY) << 1,
		RAE_E_NX_Y = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NX_Y) << 1,
		RAE_E_NX_Z = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NX_Z) << 1,
		RAE_E_NX_NY = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NX_NY) << 1,
		RAE_E_NX_NZ = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NX_NZ) << 1,
		RAE_E_NY_X = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NY_X) << 1,
		RAE_E_NY_Z = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NY_Z) << 1,
		RAE_E_NY_NX = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NY_NX) << 1,
		RAE_E_NY_NZ = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NY_NZ) << 1,
		RAE_E_NZ_X = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NZ_X) << 1,
		RAE_E_NZ_Y = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NZ_Y) << 1,
		RAE_E_NZ_NX = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NZ_NX) << 1,
		RAE_E_NZ_NY = ((_RAEElevationMode::_E << 6) | _RAEAzimuthMode::_NZ_NY) << 1,

		// 
	};

	//Projection format:
	//	X: means 0 or 1, but must at least exist a X is one
	//	_: means the bit value is used to encode information
	//	*: means still usable space
	//
	//	0000000000000000*******000000000: Not valid
	//	XXXXXXXXXXXXXXXX*******000000001: XYZ
	//	________________*******000000001: XYZ - encode UV mode (how to project to UV)
	//	XXXXXXXXXXXXXXXX*******XXXXXXXX0: RAE
	//	XXXXXXXXXXXXXXXX*******__XXXXXX0: RAE - encode elevation mode
	//	XXXXXXXXXXXXXXXX*******XX___XXX0: RAE - encode vector at (azimuth, elevation) = (0, 0)
	//	XXXXXXXXXXXXXXXX*******XXXXX___0: RAE - encode vector at (azimuth, elevation) = (90, 0)
	//	________________*******XXXXXXXX0: RAE - encode UV mode (how to project to UV)
	//
	using Projection = Flag;

	//
	template<class inType, class outType >
	outType FlagConvert(inType v);

	template<CoodSys inType, CoodSys outType >
	Eigen::Vector3d CoodConvert(const Eigen::Vector3d& coord);

	template<Projection projection>
	Eigen::Vector2d ToUV(const Eigen::Vector3d& coord);
}

#include "Common.hpp"