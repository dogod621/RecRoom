#pragma once

#include "CropperPc.h"

namespace RecRoom
{
	class CropperPcAABB : public CropperPc
	{
	public:
		CropperPcAABB(
			const Eigen::Vector3d& minAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
			const Eigen::Vector3d& maxAABB = Eigen::Vector3d(0.0, 0.0, 0.0)) 
			: CropperPc(), minAABB(minAABB), maxAABB(maxAABB) {}

	public:
		virtual void Process(const PTR(PcMED)& inV, PcIndex& outV) const;

	protected:
		Eigen::Vector3d minAABB;
		Eigen::Vector3d maxAABB;
	};
}

#include "CropperPcAABB.hpp"