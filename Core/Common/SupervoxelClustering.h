#pragma once

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/point_cloud.h>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <pcl/octree/octree_pointcloud_adjacency.h>

#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>

#include <pcl/segmentation/boost.h>
#include <pcl/octree/octree_container.h>

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	template<class PointCINS>
	class Voxel;

	template<class PointCINS>
	class Supervoxel;

	template<class PointCINS>
	class SupervoxelClustering;

	template<class PointCINS>
	class Voxel
	{
	public:
		Voxel()
			: id(-1), parent(nullptr), distance(std::numeric_limits<float>::max()),
			xyz(0.0f, 0.0f, 0.0f), rgb(0.0f, 0.0f, 0.0f), normal(0.0f, 0.0f, 0.0f, 0.0f), curvature(0.0f), diffuseAlbedo(0.0f), specularSharpness(0.0f) {}

	public:
		operator PointCINS() const;
		Voxel<PointCINS>& operator += (const Voxel<PointCINS>& voxel);
		Voxel<PointCINS>& operator += (const PointCINS& point);
		Voxel<PointCINS>& operator /= (const float& n);

	public:
		int id;
		Supervoxel<PointCINS>* parent;
		float distance;
		
		Eigen::Vector3f xyz;
		Eigen::Vector3f rgb;
		Eigen::Vector4f normal;
		float curvature;
		float diffuseAlbedo;
		float specularSharpness;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class PointCINS>
	class OATLeaf : public pcl::octree::OctreeContainerBase
	{
		template<typename T, typename U, typename V>
		friend class pcl::octree::OctreePointCloudAdjacency;

	public:
		typedef std::list<OATLeaf<PointCINS>*> NeighborListT;
		typedef typename NeighborListT::const_iterator const_iterator;
		inline const_iterator cbegin() const { return (neighbors_.begin()); }
		inline const_iterator cend() const { return (neighbors_.end()); }
		inline size_t size() const { return neighbors_.size(); }

		OATLeaf() : pcl::octree::OctreeContainerBase()
		{
			this->reset();
		}

		std::size_t getNumNeighbors() const
		{
			return neighbors_.size();
		}

		int getPointCounter() const { return num_points_; }

		Voxel<PointCINS>& getData() { return data_; }

		void setData(const Voxel<PointCINS>& data_arg) { data_ = data_arg; }

		virtual std::size_t getSize() const
		{
			return num_points_;
		}

	protected:
		typedef typename NeighborListT::iterator iterator;
		inline iterator begin() { return (neighbors_.begin()); }
		inline iterator end() { return (neighbors_.end()); }

		virtual OATLeaf<PointCINS>* deepCopy() const
		{
			OATLeaf<PointCINS> *new_container = new OATLeaf<PointCINS>;
			new_container->setNeighbors(this->neighbors_);
			new_container->setPointCounter(this->num_points_);
			return new_container;
		}

		void addPoint(const PointCINS& point)
		{
			++num_points_;
			data_ += point;
		}

		void computeData()
		{
			data_ /= (static_cast<float> (num_points_));
		}

		virtual void reset()
		{
			neighbors_.clear();
			num_points_ = 0;
			data_ = Voxel<PointCINS>();
		}

		void setPointCounter(int v) { num_points_ = v; }

		void addNeighbor(OATLeaf<PointCINS>* neighbor)
		{
			neighbors_.push_back(neighbor);
		}

		void removeNeighbor(OATLeaf<PointCINS>* neighbor)
		{
			for (iterator neighb_it = neighbors_.begin(); neighb_it != neighbors_.end(); ++neighb_it)
			{
				if (*neighb_it == neighbor)
				{
					neighbors_.erase(neighb_it);
					return;
				}
			}
		}

		void setNeighbors(const NeighborListT &neighbor_arg)
		{
			neighbors_ = neighbor_arg;
		}

	private:
		int num_points_;
		NeighborListT neighbors_;
		Voxel<PointCINS> data_;
	};

	template<class PointCINS>
	struct OATLeafCompare 
	{ 
		bool operator() (OATLeaf<PointCINS>* const &left, OATLeaf<PointCINS>* const &right) const 
		{ return left->getData().id < right->getData().id; } 
	};
	
	template<class PointCINS>
	using OATLeaves = std::set<OATLeaf<PointCINS>*, OATLeafCompare<PointCINS>>;

	template<class PointCINS>
	using OAT = pcl::octree::OctreePointCloudAdjacency<PointCINS, OATLeaf<PointCINS>>;
	
	template<class PointCINS>
	class Supervoxel
	{
	public:
		Supervoxel(uint32_t label, SupervoxelClustering<PointCINS>* parent)
			: label(label), parent(parent), centroid(), leaves() {}

		void AddLeaf(OATLeaf<PointCINS>* leaf)
		{
			leaves.insert(leaf);
			leaf->getData().parent = this;
		}

		void RemoveLeaf(OATLeaf<PointCINS>* leaf)
		{
			leaves.erase(leaf);
		}

		void Expand();

		void UpdateDistance();

		void DropMembers();

		void Update()
		{
			centroid.xyz = Eigen::Vector3f::Zero();
			centroid.rgb = Eigen::Vector3f::Zero();
			centroid.normal = Eigen::Vector4f::Zero();
			centroid.curvature = 0.0f;
			centroid.diffuseAlbedo = 0.f;
			centroid.specularSharpness = 0.f;
			for (OATLeaves<PointCINS>::iterator it = leaves.begin(); it != leaves.end(); ++it)
				centroid += (*it)->getData();
			centroid /= static_cast<float> (leaves.size());
		}

		size_t size() const { return leaves.size(); }

	public:
		uint32_t label;
		SupervoxelClustering<PointCINS>* parent;
		Voxel<PointCINS> centroid;
		OATLeaves<PointCINS> leaves;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class PointCINS>
	class SupervoxelClustering : public pcl::PCLBase<PointCINS>
	{
	public:
		friend class Supervoxel<PointCINS>;

	public:
		using pcl::PCLBase<PointCINS>::initCompute;
		using pcl::PCLBase<PointCINS>::deinitCompute;
		using pcl::PCLBase<PointCINS>::input_;

	public:
		SupervoxelClustering(float voxelResolution, float seedResolution,
			float xyzImportance = 0.4f, float rgbImportance = 0.4f, float normalImportance = 1.0f, float diffuseAlbedoImportance = 5.0f, float specularSharpnessImportance = 5.0f,
			std::size_t minSize = 1, std::size_t numIter = 0)
			: voxelResolution(voxelResolution), seedResolution(seedResolution),
			xyzImportance(xyzImportance), rgbImportance(rgbImportance), normalImportance(normalImportance), diffuseAlbedoImportance(diffuseAlbedoImportance), specularSharpnessImportance(specularSharpnessImportance), minSize(minSize), numIter(numIter),
			oat(new OAT<PointCINS>(voxelResolution)), pcCentroid(), accCentroid(new pcl::search::KdTree<PointCINS>) 
		{
			if (this->numIter == 0)
			{
				this->numIter = (2.0f * this->seedResolution / this->voxelResolution);
			}
		}

		virtual void setInputCloud(const CONST_PTR(Pc<PointCINS>)& cloud)
		{
			input_ = cloud;
			oat->setInputCloud(cloud);
		}

		virtual void Extract(Pc<PointCINS>& pcLabel);

	public:
		virtual bool PrepareForSegmentation()
		{
			if (input_->points.size() == 0)
				return (false);
			oat->addPointsFromInputCloud();

			//
			pcCentroid.reset(new Pc<PointCINS>);
			pcCentroid->resize(oat->getLeafCount());
			{
				std::vector<OATLeaf<PointCINS>*>::iterator it = oat->begin();
				Pc<PointCINS>::iterator jt = pcCentroid->begin();
				for (int idx = 0; it != oat->end(); ++it, ++jt, ++idx)
				{
					Voxel<PointCINS>& voxel = (*it)->getData();
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

		float Distance(const Voxel<PointCINS> &v1, const Voxel<PointCINS> &v2) const
		{
			float dist = xyzImportance * (v1.xyz - v2.xyz).norm() / seedResolution;

			dist += rgbImportance * ((v1.rgb / 255.0f) - (v2.rgb / 255.0f)).norm();
			dist += normalImportance * (1.0f - std::abs(v1.normal.dot(v2.normal)));
			dist += diffuseAlbedoImportance * std::abs(v1.diffuseAlbedo - v2.diffuseAlbedo) / 255.0f;
			dist += specularSharpnessImportance * std::abs(v1.specularSharpness - v2.specularSharpness);

			return  dist;
		}

	public:
		typename OAT<PointCINS>::Ptr oat;
		PTR(Pc<PointCINS>) pcCentroid;
		typename pcl::search::KdTree<PointCINS>::Ptr accCentroid;

		float voxelResolution;
		float seedResolution;

		float xyzImportance;
		float rgbImportance;
		float normalImportance;
		float diffuseAlbedoImportance;
		float specularSharpnessImportance;
		std::size_t minSize;
		std::size_t numIter;

		//Make boost::ptr_list can access the private class Supervoxel
		friend void boost::checked_delete<>(const RecRoom::Supervoxel<PointCINS>*);

		boost::ptr_list<Supervoxel<PointCINS>> supervoxels;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "SupervoxelClustering.hpp"
