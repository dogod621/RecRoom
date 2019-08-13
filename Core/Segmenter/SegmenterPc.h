#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPc 
		: public SearchInputSurfaceProcessorPc2PcInOut<PointType, PointType>
	{
	public:
		SegmenterPc(std::size_t numMaxLabels = 5)
			: SearchInputSurfaceProcessorPc2PcInOut<PointType, PointType>(), numMaxLabels(numMaxLabels)
		{
			name = "SegmenterPc";
		}

	public:
		inline virtual bool OutputPointValid(const PointType& p) const
		{
			return p.softLabelEnd > p.softLabelStart;
		}

		virtual void Process(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& outpu) const { THROW_EXCEPTION("This should not be called"); }

		virtual void ProcessInOut(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const PTR(Pc<PointType>)& inOut,
			const CONST_PTR(PcIndex)& filter) const { THROW_EXCEPTION("This should not be called"); }

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const { THROW_EXCEPTION("This should not be called"); }

		virtual void Process(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output,
			PcSoftLabel& cache) const;

		virtual void ProcessInOut(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const PTR(Pc<PointType>)& inOut,
			const CONST_PTR(PcIndex)& filter,
			PcSoftLabel& cache) const;

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output,
			PcSoftLabel& cache) const = 0;

	public:
		std::size_t getNumMaxLabels() const { return numMaxLabels; }
		void setNumMaxLabels(std::size_t v) { numMaxLabels = v; }

	protected:
		std::size_t numMaxLabels;
	};
}

#include "SegmenterPc.hpp"