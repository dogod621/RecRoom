#pragma once

#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

#include "SupervoxelClustering.h"

namespace RecRoom
{
	template<class PointCINS>
	Voxel<PointCINS>::operator PointCINS() const
	{
		PointCINS point;

		point.x = xyz[0];
		point.y = xyz[1];
		point.z = xyz[2];

		point.r = std::min(std::max(rgb[0], 0.f), 255.f);
		point.g = std::min(std::max(rgb[1], 0.f), 255.f);
		point.b = std::min(std::max(rgb[2], 0.f), 255.f);

		point.intensity = intensity;

		point.normal_x = normal[0];
		point.normal_y = normal[1];
		point.normal_z = normal[2];
		point.curvature = curvature;

		point.sharpness = sharpness;

		return point;
	}

	template<class PointCINS>
	Voxel<PointCINS>& Voxel<PointCINS>::operator += (const Voxel<PointCINS>& voxel)
	{
		xyz += voxel.xyz;

		rgb += voxel.rgb;

		intensity += voxel.intensity;

		normal += voxel.normal;
		curvature += voxel.curvature;

		sharpness += voxel.sharpness;

		return *this;
	}

	template<class PointCINS>
	Voxel<PointCINS>& Voxel<PointCINS>::operator += (const PointCINS& point)
	{
		xyz[0] += point.x;
		xyz[1] += point.y;
		xyz[2] += point.z;

		rgb[0] += point.r;
		rgb[1] += point.g;
		rgb[2] += point.b;

		intensity += point.intensity;

		normal[0] += point.normal_x;
		normal[1] += point.normal_y;
		normal[2] += point.normal_z;
		curvature += point.curvature;

		sharpness += point.sharpness;

		return *this;
	}

	template<class PointCINS>
	Voxel<PointCINS>& Voxel<PointCINS>::operator /= (const float& n)
	{
		xyz /= n;

		rgb /= n;

		intensity /= n;

		normal.normalize();
		curvature /= n;

		sharpness /= n;

		return *this;
	}

	template<class PointCINS>
	void Supervoxel<PointCINS>::Expand()
	{
		std::vector<OATLeaf<PointCINS>*> newOwned;
		newOwned.reserve(leaves.size() * 9);
		for (OATLeaves<PointCINS>::iterator it = leaves.begin(); it != leaves.end(); ++it)
		{
			//for each neighbor of the leaf
			for (OATLeaf<PointCINS>::const_iterator jt = (*it)->cbegin(); jt != (*it)->cend(); ++jt)
			{
				Voxel<PointCINS>& voxel = ((*jt)->getData());

				//TODO this is a shortcut, really we should always recompute distance
				if (voxel.parent == this) continue;

				//If distance is less than previous, we remove it from its parent's list and change the parent to this and distance (we *steal* it!)
				float dist = parent->Distance(centroid, voxel);
				if (dist < voxel.distance)
				{
					voxel.distance = dist;
					if (voxel.parent)
						(voxel.parent)->RemoveLeaf(*jt);
					voxel.parent = this;
					newOwned.push_back(*jt);
				}
			}
		}

		//Push all new owned onto the owned leaf set
		for (std::vector<OATLeaf<PointCINS>*>::iterator it = newOwned.begin(); it != newOwned.end(); ++it)
		{
			leaves.insert(*it);
		}
	}

	template<class PointCINS>
	void SupervoxelClustering<PointCINS>::Extract(Pc<PointCINS>& pcLabel)
	{
		if (!initCompute())
		{
			PRINT_WARNING("!initCompute");
			deinitCompute();
			return;
		}

		if (!PrepareForSegmentation())
		{
			PRINT_WARNING("!PrepareForSegmentation");
			deinitCompute();
			return;
		}

		{
			CreateSupervoxels(InitialSeeds());
			ExpandSupervoxels(static_cast<int> (1.8f*seedResolution / voxelResolution));
		}

		{
			for (Pc<PointCINS>::iterator it = pcLabel.begin(); it != pcLabel.end(); ++it)
			{
				it->hasLabel = -1;
				if (pcl::isFinite<PointCINS>(*it))
				{

					Voxel<PointCINS>& voxel = oat->getLeafContainerAtPoint(*it)->getData();
					if (voxel.parent)
						it->label = voxel.parent->label;
				}
			}
		}

		deinitCompute();
	}

	template<class PointCINS>
	std::vector<int> SupervoxelClustering<PointCINS>::InitialSeeds()
	{
		pcl::octree::OctreePointCloudSearch<PointCINS> seedOctree(seedResolution);
		seedOctree.setInputCloud(pcCentroid);
		seedOctree.addPointsFromInputCloud();
		std::vector<PointCINS, Eigen::aligned_allocator<PointCINS> > centers;
		int numSeeds = seedOctree.getOccupiedVoxelCenters(centers);

		std::vector<int> oriSeeds;
		{
			oriSeeds.resize(numSeeds, 0);
			std::vector<int> closestIndex;
			std::vector<float> distance;
			closestIndex.resize(1, 0);
			distance.resize(1, 0);
			accCentroid->setInputCloud(pcCentroid);

			for (int i = 0; i < numSeeds; ++i)
			{
				accCentroid->nearestKSearch(centers[i], 1, closestIndex, distance);
				oriSeeds[i] = closestIndex[0];
			}
		}

		//
		std::vector<int> seeds;
		{
			std::vector<int> neighbors;
			std::vector<float> sqrDistances;
			seeds.reserve(oriSeeds.size());
			float searchRadius = 0.5f*seedResolution;
			float minPoints = 0.05f * (searchRadius)*(searchRadius)* M_PI / (voxelResolution*voxelResolution);
			for (size_t i = 0; i < oriSeeds.size(); ++i)
			{
				int num = accCentroid->radiusSearch(oriSeeds[i], searchRadius, neighbors, sqrDistances);
				int min_index = oriSeeds[i];
				if (num > minPoints)
				{
					seeds.push_back(min_index);
				}
			}
		}

		return seeds;
	}

	template<class PointCINS>
	void SupervoxelClustering<PointCINS>::CreateSupervoxels(std::vector<int>& seeds)
	{
		supervoxels.clear();
		for (std::size_t i = 0; i < seeds.size(); ++i)
		{
			supervoxels.push_back(new Supervoxel<PointCINS>(i + 1, this));
			OATLeaf<PointCINS>* seedLeaf = oat->at(seeds[i]);
			if (seedLeaf)
			{
				supervoxels.back().AddLeaf(seedLeaf);
			}
			else
			{
				PRINT_WARNING("Could not find leaf, supervoxel will be deleted");
			}
		}
	}

	template<class PointCINS>
	void SupervoxelClustering<PointCINS>::ExpandSupervoxels(int depth)
	{
		for (int i = 1; i < depth; ++i)
		{
			//Expand the the supervoxels by one iteration
			for (boost::ptr_list<Supervoxel<PointCINS>>::iterator it = supervoxels.begin(); it != supervoxels.end(); ++it)
			{
				it->Expand();
			}

			//Update the centers to reflect new centers
			for (boost::ptr_list<Supervoxel<PointCINS>>::iterator it = supervoxels.begin(); it != supervoxels.end(); )
			{
				if (it->size() == 0)
				{
					it = supervoxels.erase(it);
				}
				else
				{
					it->Update();
					++it;
				}
			}
		}
	}
}