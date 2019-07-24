#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcNDF
	{
	public:
		using Meta = std::size_t;
		using Data = PTR(PcNDF);

	public:
		ContainerPcNDF() {}

	public:
		virtual void Merge(const CONST_PTR(PcNDF)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual Meta GetMeta(std::size_t i) const { return i; }
		virtual Data GetData(std::size_t i) const = 0;
	};
}

#include "ContainerPcNDF.hpp"