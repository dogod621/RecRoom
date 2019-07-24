#pragma once

#include "MarchingCubes.h"

namespace RecRoom
{
	template<class PointN>
	class MarchingCubesRBF : public MarchingCubes<PointN>
	{
	public:
		typedef boost::shared_ptr<MarchingCubesRBF<PointN>> Ptr;
		typedef boost::shared_ptr<const MarchingCubesRBF<PointN>> ConstPtr;

		using SurfaceReconstruction<PointN>::input_;
		using SurfaceReconstruction<PointN>::tree_;
		using MarchingCubes::grid_;
		using MarchingCubes::res_x_;
		using MarchingCubes::res_y_;
		using MarchingCubes::res_z_;
		using MarchingCubes::size_voxel_;
		using MarchingCubes::upper_boundary_;
		using MarchingCubes::lower_boundary_;

		MarchingCubesRBF(const float off_surface_epsilon = 0.1f, const float percentage_extend_grid = 0.0f, const float iso_level = 0.0f,
			int res_x = 50, int res_y = 50, int res_z = 50)
			: MarchingCubes<PointN>(percentage_extend_grid, iso_level, res_x, res_y, res_z),
			off_surface_epsilon_(off_surface_epsilon) {}

	protected:
		virtual void voxelizeData() ;

		inline double kernel(Eigen::Vector3d c, Eigen::Vector3d x)
		{
			double r = (x - c).norm();
			return (r * r * r);
		}

	protected:
		float off_surface_epsilon_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "MarchingCubesRBF.hpp"