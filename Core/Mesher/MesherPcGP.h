#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	class MesherPcGP : public MesherPc
	{
	public:
		MesherPcGP(
			double resolution,
			int maxBinarySearchLevel = 10,
			int maxNumNei = 50,
			int paddingSize = 3)
			: resolution(resolution), maxBinarySearchLevel(maxBinarySearchLevel), maxNumNei(maxNumNei), paddingSize(paddingSize),
			MesherPc() {}

	protected:
		virtual void ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const;

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