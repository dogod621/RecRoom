#include "Random.h"

namespace RecRoom
{
	std::random_device Random::randomDevice;
	std::mt19937_64 Random::generator(randomDevice());
	std::uniform_real_distribution<R32> Random::uniR32(0.f, 1.f);
	std::uniform_real_distribution<R64> Random::uniR64(0.0, 1.0);
	std::uniform_int_distribution<I16> Random::uniI16(std::numeric_limits<I16>::min(), std::numeric_limits<I16>::max());
	std::uniform_int_distribution<I32> Random::uniI32(std::numeric_limits<I32>::min(), std::numeric_limits<I32>::max());
	std::uniform_int_distribution<I64> Random::uniI64(std::numeric_limits<I64>::min(), std::numeric_limits<I64>::max());
	std::uniform_int_distribution<UI16> Random::uniUI16(std::numeric_limits<UI16>::min(), std::numeric_limits<UI16>::max());
	std::uniform_int_distribution<UI32> Random::uniUI32(std::numeric_limits<UI32>::min(), std::numeric_limits<UI32>::max());
	std::uniform_int_distribution<UI64> Random::uniUI64(std::numeric_limits<UI64>::min(), std::numeric_limits<UI64>::max());
};