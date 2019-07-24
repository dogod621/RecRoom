#pragma once

#include "MarchingCubes.h"

namespace RecRoom
{
	template<class PointN>
	class MarchingCubesHoppe : public MarchingCubes<PointN>
	{
	public:
		typedef boost::shared_ptr<MarchingCubesHoppe<PointN>> Ptr;
		typedef boost::shared_ptr<const MarchingCubesHoppe<PointN>> ConstPtr;

		using SurfaceReconstruction<PointN>::input_;
		using SurfaceReconstruction<PointN>::tree_;
		using MarchingCubes::grid_;
		using MarchingCubes::res_x_;
		using MarchingCubes::res_y_;
		using MarchingCubes::res_z_;
		using MarchingCubes::size_voxel_;
		using MarchingCubes::upper_boundary_;
		using MarchingCubes::lower_boundary_;

		MarchingCubesHoppe(const float dist_ignore = -1.0f, const float percentage_extend_grid = 0.0f, const float iso_level = 0.0f,
			int res_x = 50, int res_y = 50, int res_z = 50)
			: MarchingCubes<PointN>(percentage_extend_grid, iso_level, res_x, res_y, res_z),
			dist_ignore_(dist_ignore) {}

	protected:
		virtual void voxelizeData() ;

	protected:
		float dist_ignore_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "MarchingCubesHoppe.hpp"