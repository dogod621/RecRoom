#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	template<class PointType>
	class MesherPcGP : public MesherPc<PointType>
	{
	public:
		using Sampler = MesherPc<PointType>::Sampler;
		using Filter = MesherPc<PointType>::Filter;
		using Interpolator = MesherPc<PointType>::Interpolator;
		using InterpolatorNearest = MesherPc<PointType>::InterpolatorNearest;

	public:
		MesherPcGP(
			double resolution,
			int maxBinarySearchLevel = 10,
			int maxNumNei = 50,
			int paddingSize = 3,
			CONST_PTR(Sampler) preprocessSampler = nullptr,
			CONST_PTR(Filter) preprocessFilter = nullptr,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: resolution(resolution), maxBinarySearchLevel(maxBinarySearchLevel), maxNumNei(maxNumNei), paddingSize(paddingSize),
			MesherPc<PointType>(preprocessSampler, preprocessFilter, fieldInterpolator) 
		{
			name = "MesherPcGP";
		}

	protected:
		virtual void ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const;

	public:
		double getResolution() const { return resolution; }
		int getMaxBinarySearchLevel() const { return maxBinarySearchLevel; }
		int getMaxNumNei() const { return maxNumNei; }
		int getPaddingSize() const { return paddingSize; }

		void setResolution(double v) { resolution = v; }
		void setMaxBinarySearchLevel(int v) { maxBinarySearchLevel = v; }
		void setMaxNumNei(int v) { maxNumNei = v; }
		void setPaddingSize(int v) { paddingSize = v; }

	protected:
		// brief The size of a leaf.
		double resolution;

		// brief Max binary search level. 
		int maxBinarySearchLevel;

		// brief Number of neighbors (k) to use. 
		int maxNumNei;

		// brief Padding size. 
		int paddingSize;
	};
}

#include "MesherPcGP.hpp"