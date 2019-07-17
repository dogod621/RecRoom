#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	class MesherPcMC : public MesherPc
	{
	public:
		MesherPcMC(
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f,
			int gridRes = 50)
			: percentageExtendGrid(percentageExtendGrid),
			isoLevel(isoLevel),
			gridRes(gridRes),
			MesherPc() {}

	public:
		virtual void Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const;

	protected:
		virtual void ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const = 0;

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

	class MesherPcMCRBF : public MesherPcMC
	{
	public:
		MesherPcMCRBF(
			float offSurfaceEpsilon = 0.1f,
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f,
			int gridRes = 50)
			: offSurfaceEpsilon(offSurfaceEpsilon),
			MesherPcMC(percentageExtendGrid, isoLevel, gridRes) {}

	protected:
		virtual void ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const;

	public:
		float getOffSurfaceEpsilon() const { return offSurfaceEpsilon; }
		
		void setOffSurfaceEpsilon(float v) { offSurfaceEpsilon = v; }
		
	protected:
		float offSurfaceEpsilon;
	};

	class MesherPcMCHoppe : public MesherPcMC
	{
	public:
		MesherPcMCHoppe(
			float distIgnore = -1.0f,
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f,
			int gridRes = 50)
			: distIgnore(distIgnore),
			MesherPcMC(percentageExtendGrid, isoLevel, gridRes) {}

	protected:
		virtual void ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const;

	public:
		float getDistIgnore() const { return distIgnore; }

		void setDistIgnore(float v) { distIgnore = v; }

	protected:
		float distIgnore;
	};
}

#include "MesherPcMC.hpp"