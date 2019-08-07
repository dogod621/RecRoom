#pragma once

#include "SegmenterPc.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPcSVC : public SegmenterPc<PointType>
	{
	public:
		SegmenterPcSVC(float voxelResolution, float seedResolution, 
			float xyzImportance = 0.4f, float rgbImportance = 0.4f, float intensityImportance = 5.0f, float normalImportance = 1.0f, float sharpnessImportance = 5.0,
			std::size_t minSize = 1, std::size_t numIter = 0)
			: SegmenterPc<PointType>(), voxelResolution(voxelResolution), seedResolution(seedResolution),
			xyzImportance(xyzImportance), rgbImportance(rgbImportance), intensityImportance(intensityImportance),  normalImportance(normalImportance), sharpnessImportance(sharpnessImportance),
			minSize(minSize), numIter(numIter)
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
		float getRGBImportance() const { return rgbImportance; }
		float getIntensityImportance() const { return intensityImportance; }
		float getNormalImportance() const { return normalImportance; }
		float getSharpnessImportance() const { return sharpnessImportance; }
		std::size_t getMinSize() const { return minSize; }
		std::size_t getNumIter() const { return numIter; }

		void setVoxelResolution(float v) { voxelResolution = v; }
		void setSeedResolution(float v) { seedResolution = v; }
		float setXYZImportance(float v)  { xyzImportance = v; }
		float setRGBImportance(float v)  { rgbImportance = v; }
		void setIntensityImportance(float v) { intensityImportance = v; }
		void setNormalImportance(float v) { normalImportance = v; }
		void setSharpnessImportance(float v) { sharpnessImportance = v; }
		void setMinSize(std::size_t v) { minSize = v; }
		void setNumIter(std::size_t v) { numIter = v; }

	protected:
		float voxelResolution;
		float seedResolution;

		float xyzImportance;
		float rgbImportance;
		float intensityImportance;
		float normalImportance;
		float sharpnessImportance;

		std::size_t minSize;
		std::size_t numIter;
	};
}

#include "SegmenterPcSVC.hpp"