#pragma once

#include <pcl/surface/boost.h>
#include <pcl/surface/reconstruction.h>

#include "Point.h"

namespace RecRoom
{
	template<class PointNT>
	class MarchingCubes : public pcl::SurfaceReconstruction<PointNT>
	{
	public:
		typedef boost::shared_ptr<MarchingCubes<PointNT>> Ptr;
		typedef boost::shared_ptr<const MarchingCubes<PointNT>> ConstPtr;

		using SurfaceReconstruction<PointNT>::input_;
		using SurfaceReconstruction<PointNT>::tree_;

		MarchingCubes(const float percentage_extend_grid = 0.0f, const float iso_level = 0.0f,
			int res_x = 50, int res_y = 50, int res_z = 50)
			: percentage_extend_grid_(percentage_extend_grid), iso_level_(iso_level), res_x_(res_x), res_y_(res_y), res_z_(res_z),
			pcl::SurfaceReconstruction<PointNT>()
		{
			if (!(iso_level_ >= 0 && iso_level_ < 1))
				THROW_EXCEPTION("iso_level is not valid");
		}

	protected:
		virtual void voxelizeData() = 0;

		inline void interpolateEdge(Eigen::Vector3f &p1, Eigen::Vector3f &p2, float val_p1, float val_p2, Eigen::Vector3f &output)
		{
			const float mu = (iso_level_ - val_p1) / (val_p2 - val_p1);
			output = p1 + mu * (p2 - p1);
		}

		void createSurface(const std::vector<float>& leaf_node, const Eigen::Vector3i& index_3d, Pc<PointNT>& cloud);

		virtual float getGridValue(Eigen::Vector3i pos)
		{
			/// TODO what to return?
			if (pos[0] < 0 || pos[0] >= res_x_) return -1.0f;
			if (pos[1] < 0 || pos[1] >= res_y_) return -1.0f;
			if (pos[2] < 0 || pos[2] >= res_z_) return -1.0f;
			return grid_[pos[0] * res_y_*res_z_ + pos[1] * res_z_ + pos[2]];
		}

		void getNeighborList1D(std::vector<float> &leaf, Eigen::Vector3i &index3d);

		std::string getClassName() const { return ("MarchingCubes"); }

		virtual void performReconstruction(pcl::PolygonMesh &output)
		{
			Pc<PointNT> points;
			performReconstruction(points, output.polygons);
			pcl::toPCLPointCloud2(points, output.cloud);
		}

		virtual void performReconstruction(Pc<PointNT>& points, std::vector<pcl::Vertices>& polygons);

	protected:
		std::vector<float> grid_;
		int res_x_, res_y_, res_z_;

		Eigen::Array3f upper_boundary_;
		Eigen::Array3f lower_boundary_;
		Eigen::Array3f size_voxel_;
		float percentage_extend_grid_;
		float iso_level_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "MarchingCubes.hpp"