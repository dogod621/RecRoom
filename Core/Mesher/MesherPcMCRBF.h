#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	class MesherPcMCRBF : public MesherPc
	{
	public:
		MesherPcMCRBF(
			float offSurfaceEpsilon = 0.1f,
			float percentageExtendGrid = 0.0f,
			float isoLevel = 0.0f) 
			: offSurfaceEpsilon(offSurfaceEpsilon),
			percentageExtendGrid(percentageExtendGrid),
			isoLevel(isoLevel),
			MesherPc() {}

	public:
		virtual void Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const;

	public:
		float getOffSurfaceEpsilon() const { return offSurfaceEpsilon; }
		float getPercentageExtendGrid() const { return percentageExtendGrid; }
		float getISOLevel() const { return isoLevel; }

		void setOffSurfaceEpsilon(float v) { offSurfaceEpsilon = v; }
		void setPercentageExtendGrid(float v) { percentageExtendGrid = v; }
		void setISOLevel(float v) { isoLevel = v; }

	protected:
		float offSurfaceEpsilon;
		float percentageExtendGrid;
		float isoLevel;
	};
}

#include "MesherPcMCRBF.hpp"