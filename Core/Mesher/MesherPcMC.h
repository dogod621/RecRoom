#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	template<class PointType>
	class MesherPcMC : public MesherPc<PointType>
	{
	public:
		using Interpolator = MesherPc<PointType>::Interpolator;
		using InterpolatorNearest = MesherPc<PointType>::InterpolatorNearest;

	public:
		MesherPcMC(
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f,
			int gridRes = 50,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: percentageExtendGrid(percentageExtendGrid),
			isoLevel(isoLevel),
			gridRes(gridRes),
			MesherPc<PointType>(fieldInterpolator) 
		{
			name = "MesherPcGP3";
		}

	public:
		float getPercentageExtendGrid() const { return percentageExtendGrid; }
		float getISOLevel() const { return isoLevel; }
		int getGridRes() const { return gridRes; }

		void setPercentageExtendGrid(float v) { percentageExtendGrid = v; }
		void setISOLevel(float v) { isoLevel = v; }
		void setGridRes(int v) { gridRes = v; }

	protected:
		float percentageExtendGrid;
		float isoLevel;
		int gridRes;
	};

	template<class PointType>
	class MesherPcMCRBF : public MesherPcMC<PointType>
	{
	public:
		using Interpolator = MesherPcMC<PointType>::Interpolator;
		using InterpolatorNearest = MesherPcMC<PointType>::InterpolatorNearest;

	public:
		MesherPcMCRBF(
			float offSurfaceEpsilon = 0.1f,
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f,
			int gridRes = 50,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: offSurfaceEpsilon(offSurfaceEpsilon),
			MesherPcMC<PointType>(percentageExtendGrid, isoLevel, gridRes, fieldInterpolator) {}

	protected:
		virtual void ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const;

	public:
		float getOffSurfaceEpsilon() const { return offSurfaceEpsilon; }
		
		void setOffSurfaceEpsilon(float v) { offSurfaceEpsilon = v; }
		
	protected:
		float offSurfaceEpsilon;
	};

	template<class PointType>
	class MesherPcMCHoppe : public MesherPcMC<PointType>
	{
	public:
		using Interpolator = MesherPcMC<PointType>::Interpolator;
		using InterpolatorNearest = MesherPcMC<PointType>::InterpolatorNearest;

	public:
		MesherPcMCHoppe(
			float distIgnore = -1.0f,
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f,
			int gridRes = 50,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: distIgnore(distIgnore),
			MesherPcMC<PointType>(percentageExtendGrid, isoLevel, gridRes, fieldInterpolator) {}

	protected:
		virtual void ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const;

	public:
		float getDistIgnore() const { return distIgnore; }

		void setDistIgnore(float v) { distIgnore = v; }

	protected:
		float distIgnore;
	};
}

#include "MesherPcMC.hpp"