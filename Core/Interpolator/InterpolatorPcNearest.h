#pragma once

#include "InterpolatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class InterpolatorPcNearest : public InterpolatorPc<InPointType, OutPointType>, public ThreadAble
	{
	public:
		static void InterpolationTask(
			int id,
			void* self,
			void* searchSurface,
			void* input,
			void* filter,
			void* output);

	public:
		InterpolatorPcNearest() : ThreadAble(), InterpolatorPc<InPointType, OutPointType>()
		{
			name = "InterpolatorPcNearest";
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<InPointType>)& searchSurface,
			const CONST_PTR(Pc<InPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutPointType>& output) const;

		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			PRINT_WARNING("not used");
			return true;
		}

		inline virtual bool InputPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}
	};
}

#include "InterpolatorPcNearest.hpp"