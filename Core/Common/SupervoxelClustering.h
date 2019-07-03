#pragma once

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/boost.h>

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	class Voxel;
	class Supervoxel;
	class SupervoxelClustering;

	class Voxel
	{
	public:
		Voxel()
			: id(-1), parent(nullptr), distance(std::numeric_limits<float>::max()),
			xyz(0.0f, 0.0f, 0.0f), normal(0.0f, 0.0f, 0.0f, 0.0f), curvature(0.0f), rgb(0.0f, 0.0f, 0.0f), intensity(0.0f) {}

	public:
		operator PointMED() const;
		Voxel& operator += (const Voxel& voxel);
		Voxel& operator += (const PointMED& point);
		Voxel& operator /= (const float& n);

	public:
		int id;
		Supervoxel* parent;
		float distance;
		
		Eigen::Vector3f xyz;
		Eigen::Vector4f normal;
		float curvature;
		Eigen::Vector3f rgb;
		float intensity;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	using OATLeaf = pcl::octree::OctreePointCloudAdjacencyContainer<PointMED, Voxel>;
	struct OATLeafCompare { bool operator() (OATLeaf* const &left, OATLeaf* const &right) const { return left->getData().id < right->getData().id; } };
	using OATLeaves = std::set<OATLeaf*, OATLeafCompare>;
	using OAT = pcl::octree::OctreePointCloudAdjacency<PointMED, OATLeaf>;
	
	class Supervoxel
	{
	public:
		Supervoxel(uint32_t label, SupervoxelClustering* parent)
			: label(label), parent(parent), centroid(), leaves() {}

		void AddLeaf(OATLeaf* leaf)
		{
			leaves.insert(leaf);
			leaf->getData().parent = this;
		}

		void RemoveLeaf(OATLeaf* leaf)
		{
			leaves.erase(leaf);
		}

		void Expand();

		void Update()
		{
			centroid.xyz = Eigen::Vector3f::Zero();
			centroid.normal = Eigen::Vector4f::Zero();
			centroid.curvature = 0.0f;
			centroid.rgb = Eigen::Vector3f::Zero();
			centroid.intensity = 0.f;
			for (OATLeaves::iterator it = leaves.begin(); it != leaves.end(); ++it)
				centroid += (*it)->getData();
			centroid /= static_cast<float> (leaves.size());
		}

		size_t size() const { return leaves.size(); }

	public:
		uint32_t label;
		SupervoxelClustering* parent;
		Voxel centroid;
		OATLeaves leaves;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	class SupervoxelClustering : public pcl::PCLBase<PointMED>
	{
	public:
		friend class Supervoxel;

	public:
		using pcl::PCLBase<PointMED>::initCompute;
		using pcl::PCLBase<PointMED>::deinitCompute;
		using pcl::PCLBase<PointMED>::input_;

	public:
		SupervoxelClustering(float voxelResolution, float seedResolution,
			float xyzImportance = 0.4f, float normalImportance = 1.0f, float rgbImportance = 0.4f, float intensityImportance = 5.0f)
			: voxelResolution(voxelResolution), seedResolution(seedResolution),
			xyzImportance(xyzImportance), normalImportance(normalImportance), rgbImportance(rgbImportance), intensityImportance(intensityImportance),
			oat(new OAT(voxelResolution)), pcCentroid(), accCentroid(new pcl::search::KdTree<PointMED>) {}

		virtual void setInputCloud(const CONST_PTR(PcMED)& cloud)
		{
			input_ = cloud;
			oat->setInputCloud(cloud);
		}

		virtual void Extract(PcMED& pcLabel);

	public:
		virtual bool PrepareForSegmentation()
		{
			if (input_->points.size() == 0)
				return (false);
			oat->addPointsFromInputCloud();

			//
			pcCentroid.reset(new PcMED);
			pcCentroid->resize(oat->getLeafCount());
			{
				std::vector<OATLeaf*>::iterator it = oat->begin();
				PcMED::iterator jt = pcCentroid->begin();
				for (int idx = 0; it != oat->end(); ++it, ++jt, ++idx)
				{
					Voxel& voxel = (*it)->getData();
					voxel.id = idx;
					(*jt) = voxel;
				}
			}

			//
			return true;
		}

		std::vector<int> InitialSeeds();

		void CreateSupervoxels(std::vector<int>& seeds);
		void ExpandSupervoxels(int depth);

		float Distance(const Voxel &v1, const Voxel &v2) const
		{
			float dist = xyzImportance * (v1.xyz - v2.xyz).norm() / seedResolution;

#ifdef POINT_MED_WITH_NORMAL
			dist += normalImportance * (1.0f - std::abs(v1.normal.dot(v2.normal)));
#endif

#ifdef POINT_MED_WITH_RGB
			dist += rgbImportance * ((v1.rgb / 255.0f) - (v2.rgb / 255.0f)).norm();
#endif

#ifdef POINT_MED_WITH_INTENSITY
			dist += intensityImportance * std::abs(v1.intensity - v2.intensity) / 255.0f;
#endif

			return  dist;
		}

	public:
		typename OAT::Ptr oat;
		PTR(PcMED) pcCentroid;
		typename pcl::search::KdTree<PointMED>::Ptr accCentroid;

		float voxelResolution;
		float seedResolution;

		float xyzImportance;
		float normalImportance;
		float rgbImportance;
		float intensityImportance;

		//Make boost::ptr_list can access the private class Supervoxel
		friend void boost::checked_delete<>(const RecRoom::Supervoxel*);

		boost::ptr_list<Supervoxel> supervoxels;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "SupervoxelClustering.hpp"
