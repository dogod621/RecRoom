#pragma once

#include "SegmenterPc.h"

namespace RecRoom
{
	class SegmenterPcSVC : public SegmenterPc
	{
	public:
		SegmenterPcSVC(float voxelResolution, float seedResolution, 
			float xyzImportance = 0.4f, float normalImportance = 1.0f, float rgbImportance = 0.1f, float intensityImportance = 1.0f)
			: SegmenterPc(), voxelResolution(voxelResolution), seedResolution(seedResolution), 
			xyzImportance(xyzImportance), normalImportance(normalImportance), rgbImportance(rgbImportance), intensityImportance(intensityImportance) {}

	public:
		virtual void Process(const PTR(PcMED)& pc) const;

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