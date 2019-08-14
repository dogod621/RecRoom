#pragma once

#include "SegmenterPc.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPcSVC : public SegmenterPc<PointType>
	{
	public:
		SegmenterPcSVC(float voxelResolution, float seedResolution, 
			float xyzImportance = 0.4f, float rgbImportance = 0.4f, float normalImportance = 1.0f, float diffuseAlbedoImportance = 5.0f, float specularSharpnessImportance = 0.0,
			float weightSmoothParm = 2.0, std::size_t numMaxLabels = 5,
			std::size_t minSize = 1, std::size_t numIter = 0)
			: SegmenterPc<PointType>(numMaxLabels), voxelResolution(voxelResolution), seedResolution(seedResolution),
			xyzImportance(xyzImportance), rgbImportance(rgbImportance),  normalImportance(normalImportance), diffuseAlbedoImportance(diffuseAlbedoImportance), specularSharpnessImportance(specularSharpnessImportance),
			weightSmoothParm(weightSmoothParm),
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
			Pc<PointType>& output,
			PcSoftLabel& cache) const;

	public:
		float getVoxelResolution() const { return voxelResolution; }
		float getSeedResolution() const { return seedResolution; }
		float getXYZImportance() const { return xyzImportance; }
		float getRGBImportance() const { return rgbImportance; }
		float getNormalImportance() const { return normalImportance; }
		float getDiffuseAlbedoImportance() const { return diffuseAlbedoImportance; }
		float getSpecularSharpnessImportance() const { return specularSharpnessImportance; }
		float getWeightSmoothParm() const { return weightSmoothParm; }
		std::size_t getMinSize() const { return minSize; }
		std::size_t getNumIter() const { return numIter; }

		void setVoxelResolution(float v) { voxelResolution = v; }
		void setSeedResolution(float v) { seedResolution = v; }
		float setXYZImportance(float v)  { xyzImportance = v; }
		float setRGBImportance(float v)  { rgbImportance = v; }
		void setNormalImportance(float v) { normalImportance = v; }
		void setDiffuseAlbedoImportance(float v) { diffuseAlbedoImportance = v; }
		void setSpecularSharpnessImportance(float v) { specularSharpnessImportance = v; }
		void setWeightSmoothParm(float v) { weightSmoothParm = v; }
		void setMinSize(std::size_t v) { minSize = v; }
		void setNumIter(std::size_t v) { numIter = v; }

	protected:
		float voxelResolution;
		float seedResolution;

		float xyzImportance;
		float rgbImportance;
		float normalImportance;
		float diffuseAlbedoImportance;
		float specularSharpnessImportance;
		float weightSmoothParm;

		std::size_t minSize;
		std::size_t numIter;
	};
}

#include "SegmenterPcSVC.hpp"