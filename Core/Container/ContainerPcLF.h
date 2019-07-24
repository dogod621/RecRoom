#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcLF
	{
	public:
		struct Meta
		{ // Not done
		};

		struct Data : public Meta
		{ // Not done
		};

	public:
		ContainerPcLF() {}

	public:
		virtual void Merge(const CONST_PTR(PcLF)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual Meta GetData(std::size_t i) const = 0;
		virtual Data GetMeta(std::size_t i) const = 0;
	};
}

#include "ContainerPcLF.hpp"