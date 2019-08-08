#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcNDF
	{
	public:
		using Data = PTR(PcNDF);

	public:
		ContainerPcNDF() {}

	public:
		virtual void Merge(const CONST_PTR(PcNDF)& v) = 0;
		virtual std::size_t NumLabel() const = 0;
		virtual std::size_t NumSerialNumber() const = 0;
		virtual Data GetData(std::size_t label, std::size_t serialNumber) const = 0;
		virtual Data GetData(std::size_t label) const = 0;
	};
}

#include "ContainerPcNDF.hpp"