#pragma once

#include "Random.h"

// 
namespace RecRoom
{
	template<>
	static inline R32 Random::Uniform<R32>()
	{
		return uniR32(generator);
	}

	template<>
	static inline Eigen::Vector2f Random::Uniform<Eigen::Vector2f>()
	{
		return Eigen::Vector2f(uniR32(generator), uniR32(generator));
	}

	template<>
	static inline Eigen::Vector3f Random::Uniform<Eigen::Vector3f>()
	{
		return Eigen::Vector3f(uniR32(generator), uniR32(generator), uniR32(generator));
	}

	template<>
	static inline Eigen::Vector4f Random::Uniform<Eigen::Vector4f>()
	{
		return Eigen::Vector4f(uniR32(generator), uniR32(generator), uniR32(generator), uniR32(generator));
	}

	template<>
	static inline R64 Random::Uniform<R64>()
	{
		return uniR64(generator);
	}

	template<>
	static inline Eigen::Vector2d Random::Uniform<Eigen::Vector2d>()
	{
		return Eigen::Vector2d(uniR64(generator), uniR64(generator));
	}

	template<>
	static inline Eigen::Vector3d Random::Uniform<Eigen::Vector3d>()
	{
		return Eigen::Vector3d(uniR64(generator), uniR64(generator), uniR64(generator));
	}

	template<>
	static inline Eigen::Vector4d Random::Uniform<Eigen::Vector4d>()
	{
		return Eigen::Vector4d(uniR64(generator), uniR64(generator), uniR64(generator), uniR64(generator));
	}

	template<>
	static inline I16 Random::Uniform<I16>()
	{
		return uniI16(generator);
	}

	template<>
	static inline I32 Random::Uniform<I32>()
	{
		return uniI32(generator);
	}

	template<>
	static inline Eigen::Vector2i Random::Uniform<Eigen::Vector2i>()
	{
		return Eigen::Vector2i(uniI32(generator), uniI32(generator));
	}

	template<>
	static inline Eigen::Vector3i Random::Uniform<Eigen::Vector3i>()
	{
		return Eigen::Vector3i(uniI32(generator), uniI32(generator), uniI32(generator));
	}

	template<>
	static inline Eigen::Vector4i Random::Uniform<Eigen::Vector4i>()
	{
		return Eigen::Vector4i(uniI32(generator), uniI32(generator), uniI32(generator), uniI32(generator));
	}

	template<>
	static inline I64 Random::Uniform<I64>()
	{
		return uniI64(generator);
	}

	template<>
	static inline UI16 Random::Uniform<UI16>()
	{
		return uniUI16(generator);
	}

	template<>
	static inline UI32 Random::Uniform<UI32>()
	{
		return uniUI32(generator);
	}

	template<>
	static inline UI64 Random::Uniform<UI64>()
	{
		return uniUI64(generator);
	}

	template<class T>
	static inline T Random::UniformReal(T minV, T maxV)
	{
		std::uniform_real_distribution<T> uniR(minV, maxV);
		return uniR(generator);
	}

	template<class T>
	static inline T Random::UniformInt(T minV, T maxV)
	{
		std::uniform_int_distribution<T> uniI(minV, maxV);
		return uniI(generator);
	}
};

