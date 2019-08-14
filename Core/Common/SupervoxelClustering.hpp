#pragma once

#include <numeric>      // std::iota
#include <algorithm>    // std::sort

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

		point.normal_x = normal[0];
		point.normal_y = normal[1];
		point.normal_z = normal[2];
		point.curvature = curvature;

		point.diffuseAlbedo = diffuseAlbedo;

		point.specularSharpness = specularSharpness;

		return point;
	}

	template<class PointCINS>
	Voxel<PointCINS>& Voxel<PointCINS>::operator += (const Voxel<PointCINS>& voxel)
	{
		xyz += voxel.xyz;

		rgb += voxel.rgb;

		normal += voxel.normal;
		curvature += voxel.curvature;

		diffuseAlbedo += voxel.diffuseAlbedo;

		specularSharpness += voxel.specularSharpness;

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

		normal[0] += point.normal_x;
		normal[1] += point.normal_y;
		normal[2] += point.normal_z;
		curvature += point.curvature;

		diffuseAlbedo += point.diffuseAlbedo;

		specularSharpness += point.specularSharpness;

		return *this;
	}

	template<class PointCINS>
	Voxel<PointCINS>& Voxel<PointCINS>::operator /= (const float& n)
	{
		xyz /= n;

		rgb /= n;

		normal.normalize();
		curvature /= n;

		diffuseAlbedo /= n;

		specularSharpness /= n;

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

				if (voxel.parent == this)
				{
					continue; //TODO this is a shortcut, really we should always recompute distance
				}
				else
				{
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
		}

		//Push all new owned onto the owned leaf set
		for (std::vector<OATLeaf<PointCINS>*>::iterator it = newOwned.begin(); it != newOwned.end(); ++it)
		{
			leaves.insert(*it);
		}
	}

	template<class PointCINS>
	void Supervoxel<PointCINS>::DropMembers()
	{
		for (OATLeaves<PointCINS>::iterator it = leaves.begin(); it != leaves.end(); ++it)
		{
			Voxel<PointCINS>& voxel = ((*it)->getData());
			voxel.distance = std::numeric_limits<float>::max();

			for (boost::ptr_list<Supervoxel<PointCINS>>::iterator jt = parent->supervoxels.begin(); jt != parent->supervoxels.end(); ++jt)
			{
				if (&(*jt) != this)
				{
					float dist = parent->Distance(jt->centroid, voxel);
					if (dist < voxel.distance)
					{
						voxel.distance = dist;
						if (voxel.parent != this)
							(voxel.parent)->RemoveLeaf(*it);
						voxel.parent = &(*jt);
						jt->leaves.insert(*it);
					}
				}
			}
		}
	}

	template<class PointCINS>
	void Supervoxel<PointCINS>::UpdateDistance()
	{
		for (OATLeaves<PointCINS>::iterator it = leaves.begin(); it != leaves.end(); ++it)
		{
			Voxel<PointCINS>& voxel = ((*it)->getData());
			voxel.distance = parent->Distance(centroid, voxel);
		}
	}

	template <typename T>
	std::vector<std::size_t> SortIndexes(const std::vector<T> &v)
	{

		// initialize original index locations
		std::vector<std::size_t> idx(v.size());
		std::iota(idx.begin(), idx.end(), 0);

		// sort indexes based on comparing values in v
		std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });

		return idx;
	}

	template<class PointCINS>
	void SupervoxelClustering<PointCINS>::Extract(Pc<PointCINS>& cloud, PcSoftLabel& pcSoftLabel, float weightSmoothParm, std::size_t numMaxLabels)
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
			ExpandSupervoxels(numIter);
		}

		std::size_t counter = 0;
		for (boost::ptr_list<Supervoxel<PointCINS>>::iterator it = supervoxels.begin(); it != supervoxels.end(); ++it)
		{
			counter++;
		}

		//
		std::size_t numMaxLabels2 = std::min(numMaxLabels, counter);
		std::vector<float> tempLabelDistance(counter);

		std::map<OATLeaf<PointCINS>*, std::vector<SoftLabel>> vSoftLabels;
		for (std::vector<OATLeaf<PointCINS>*>::iterator it = oat->begin(); it != oat->end(); ++it)
		{
			std::vector<SoftLabel>& vSoftLabel = vSoftLabels[(*it)];
			vSoftLabel.resize(numMaxLabels2);

			Voxel<PointCINS>& voxel = (*it)->getData();

			counter = 0;
			for (boost::ptr_list<Supervoxel<PointCINS>>::iterator jt = supervoxels.begin(); jt != supervoxels.end(); ++jt)
			{
				tempLabelDistance[counter] = std::min(std::max(Distance(jt->centroid, voxel), 1e-4f), 1e4f);
				if (!pcl_isfinite(tempLabelDistance[counter]))
				{
					THROW_EXCEPTION("tempLabelDistance[counter] is not valid");
				}
				counter++;
			}

			std::vector<std::size_t> tempLabel = SortIndexes<float>(tempLabelDistance);

			// Inverse distance weighting
			// https://en.wikipedia.org/wiki/Inverse_distance_weighting
			float sumWeight = 0.0f;
			for (std::size_t i = 0; i < numMaxLabels2; ++i)
			{
				vSoftLabel[i].label = tempLabel[i];
				float weight = std::min(std::max(1.0f / std::pow(tempLabelDistance[i], weightSmoothParm), 1e-4f), 1e4f);
				vSoftLabel[i].weight = weight;
				sumWeight += weight;
			}

			if (sumWeight > 0.0f)
			{
				for (std::size_t i = 0; i < numMaxLabels2; ++i)
				{
					vSoftLabel[i].weight /= sumWeight;

					if(!pcl::isFinite(vSoftLabel[i]))
					{ 
						THROW_EXCEPTION("vSoftLabel is not valid");
					}
				}
			}
			else
			{
				THROW_EXCEPTION("sumWeight is zero");
			}
		}


		//
		pcSoftLabel.clear();
		pcSoftLabel.reserve(cloud.size() * numMaxLabels2);
		for (int px = 0; px < cloud.size(); ++px)
		{
			PointCINS& p = cloud[px];
			std::vector<SoftLabel>& vSoftLabel = vSoftLabels[oat->getLeafContainerAtPoint(p)];
			p.softLabelStart = pcSoftLabel.size();

			for (std::size_t i = 0; i < vSoftLabel.size(); ++i)
			{
				pcSoftLabel.push_back(vSoftLabel[i]);
			}

			p.softLabelEnd = pcSoftLabel.size();
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
			float searchRadius = 0.5f * seedResolution;
			float minPoints = 0.05f * (searchRadius) * (searchRadius)* M_PI / (voxelResolution * voxelResolution);
			for (size_t i = 0; i < oriSeeds.size(); ++i)
			{
				if (accCentroid->radiusSearch(oriSeeds[i], searchRadius, neighbors, sqrDistances) > minPoints)
				{
					seeds.push_back(oriSeeds[i]);
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

			// Update Distance
			for (boost::ptr_list<Supervoxel<PointCINS>>::iterator it = supervoxels.begin(); it != supervoxels.end(); ++it)
			{
				it->UpdateDistance();
			}
		}

		for (boost::ptr_list<Supervoxel<PointCINS>>::iterator it = supervoxels.begin(); it != supervoxels.end();)
		{
			if (it->size() < minSize)
			{
				it->DropMembers();
				it = supervoxels.erase(it);

				// Update Distance
				for (boost::ptr_list<Supervoxel<PointCINS>>::iterator jt = supervoxels.begin(); jt != supervoxels.end(); ++jt)
				{
					jt->UpdateDistance();
				}
			}
			else
				++it;
		}
	}
}