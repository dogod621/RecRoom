#pragma once

#include <random>

#include "Common.h"

// 
namespace RecRoom
{
	using R32 = float;
	using R64 = double;

	using I8 = char;
	using I16 = short int;
	using I32 = int;
	using I64 = long long int;

	using UI8 = unsigned char;
	using UI16 = unsigned short int;
	using UI32 = unsigned int;
	using UI64 = unsigned long long int;

	// Rand
	class Random
	{
	public:
		template<class T>
		static inline T Uniform();

		template<class T>
		static inline T UniformReal(T minV, T maxV);

		template<class T>
		static inline T UniformInt(T minV, T maxV);

	protected:
		static std::random_device randomDevice;
		static std::mt19937_64 generator;
		static std::uniform_real_distribution<R32> uniR32;
		static std::uniform_real_distribution<R64> uniR64;
		static std::uniform_int_distribution<I16> uniI16;
		static std::uniform_int_distribution<I32> uniI32;
		static std::uniform_int_distribution<I64> uniI64;
		static std::uniform_int_distribution<UI16> uniUI16;
		static std::uniform_int_distribution<UI32> uniUI32;
		static std::uniform_int_distribution<UI64> uniUI64;
	};
};

#include "Random.hpp"
