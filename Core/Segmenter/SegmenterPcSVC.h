#pragma once

#include "SegmenterPc.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPcSVC : public SegmenterPc<PointType>
	{
	public:
		SegmenterPcSVC(float voxelResolution, float seedResolution, 
			float xyzImportance = 0.4f, float normalImportance = 1.0f, float rgbImportance = 0.4f, float intensityImportance = 5.0f)
			: SegmenterPc<PointType>(), voxelResolution(voxelResolution), seedResolution(seedResolution),
			xyzImportance(xyzImportance), normalImportance(normalImportance), rgbImportance(rgbImportance), intensityImportance(intensityImportance) 
		{
			name = "SegmenterPc";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const
		{
			if (searchSurface)
				PRINT_WARNING("searchSurface is not used");
			return true;
		}

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const;

	public:
		float getVoxelResolution() const { return voxelResolution; }
		float getSeedResolution() const { return seedResolution; }
		float getXYZImportance() const { return xyzImportance; }
		float getNormalImportance() const { return normalImportance; }
		float getRGBImportance() const { return rgbImportance; }
		float getIntensityImportance() const { return intensityImportance; }

		void setVoxelResolution(float v) { voxelResolution = v; }
		void setSeedResolution(float v) { seedResolution = v; }
		float setXYZImportance(float v)  { xyzImportance = v; }
		void setNormalImportance(float v) { normalImportance = v; }
		float setRGBImportance(float v)  { rgbImportance = v; }
		void setIntensityImportance(float v) { intensityImportance = v; }

	protected:
		float voxelResolution;
		float seedResolution;

		float xyzImportance;
		float normalImportance;
		float rgbImportance;
		float intensityImportance;
	};
}

#include "SegmenterPcSVC.hpp"