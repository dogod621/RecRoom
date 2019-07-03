#include <pcl/features/normal_3d.h>

#include "SupervoxelClustering.h"

namespace RecRoom
{
	Voxel::operator PointMED() const
	{
		PointMED point;

		point.x = xyz[0];
		point.y = xyz[1];
		point.z = xyz[2];

#ifdef POINT_MED_WITH_NORMAL
		point.normal_x = normal[0];
		point.normal_y = normal[1];
		point.normal_z = normal[2];
		point.curvature = curvature;
#endif

#ifdef POINT_MED_WITH_RGB
		point.r = std::min(std::max(rgb[0], 0.f), 255.f);
		point.g = std::min(std::max(rgb[1], 0.f), 255.f);
		point.b = std::min(std::max(rgb[2], 0.f), 255.f);
#endif

#ifdef POINT_MED_WITH_INTENSITY
		point.intensity = intensity;
#endif

		return point;
	}

	Voxel& Voxel::operator += (const Voxel& voxel)
	{
		xyz += voxel.xyz;

#ifdef POINT_MED_WITH_NORMAL
		normal += voxel.normal;
		curvature += voxel.curvature;
#endif

#ifdef POINT_MED_WITH_RGB
		rgb += voxel.rgb;
#endif

#ifdef POINT_MED_WITH_INTENSITY
		intensity += voxel.intensity;
#endif
		return *this;
	}

	Voxel& Voxel::operator += (const PointMED& point)
	{
		xyz[0] += point.x;
		xyz[1] += point.y;
		xyz[2] += point.z;

#ifdef POINT_MED_WITH_NORMAL
		normal[0] += point.normal_x;
		normal[1] += point.normal_y;
		normal[2] += point.normal_z;
		curvature += point.curvature;
#endif

#ifdef POINT_MED_WITH_RGB
		rgb[0] += point.r;
		rgb[1] += point.g;
		rgb[2] += point.b;
#endif

#ifdef POINT_MED_WITH_INTENSITY
		intensity += point.intensity;
#endif
		return *this;
	}

	Voxel& Voxel::operator /= (const float& n)
	{
		xyz /=n;

#ifdef POINT_MED_WITH_NORMAL
		normal.normalize();
		curvature /= n;
#endif

#ifdef POINT_MED_WITH_RGB
		rgb /= n;
#endif

#ifdef POINT_MED_WITH_INTENSITY
		intensity /= n;
#endif
		return *this;
	}

	void Supervoxel::Expand()
	{
		std::vector<OATLeaf*> newOwned;
		newOwned.reserve(leaves.size() * 9);
		for (OATLeaves::iterator it = leaves.begin(); it != leaves.end(); ++it)
		{
			//for each neighbor of the leaf
			for (OATLeaf::const_iterator jt = (*it)->cbegin(); jt != (*it)->cend(); ++jt)
			{
				Voxel& voxel = ((*jt)->getData());

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
		for (std::vector<OATLeaf*>::iterator it = newOwned.begin(); it != newOwned.end(); ++it)
		{
			leaves.insert(*it);
		}
	}

	void SupervoxelClustering::Extract(PcMED& pcLabel)
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

#ifdef POINT_MED_WITH_SEGLABEL
		{
			for (PcMED::iterator it = pcLabel.begin(); it != pcLabel.end(); ++it)
			{
				it->hasSegLabel = -1;
				if (pcl::isFinite<PointMED>(*it))
				{
					
					Voxel& voxel = oat->getLeafContainerAtPoint(*it)->getData();
					if (voxel.parent)
						it->segLabel = voxel.parent->label;
				}
			}
		}
#endif
		deinitCompute();
	}

	std::vector<int> SupervoxelClustering::InitialSeeds()
	{
		pcl::octree::OctreePointCloudSearch<PointMED> seedOctree(seedResolution);
		seedOctree.setInputCloud(pcCentroid);
		seedOctree.addPointsFromInputCloud();
		std::vector<PointMED, Eigen::aligned_allocator<PointMED> > centers;
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

	void SupervoxelClustering::CreateSupervoxels(std::vector<int>& seeds)
	{
		supervoxels.clear();
		for (std::size_t i = 0; i < seeds.size(); ++i)
		{
			supervoxels.push_back(new Supervoxel(i + 1, this));
			OATLeaf* seedLeaf = oat->at(seeds[i]);
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

	void SupervoxelClustering::ExpandSupervoxels(int depth)
	{
		for (int i = 1; i < depth; ++i)
		{
			//Expand the the supervoxels by one iteration
			for (boost::ptr_list<Supervoxel>::iterator it = supervoxels.begin(); it != supervoxels.end(); ++it)
			{
				it->Expand();
			}

			//Update the centers to reflect new centers
			for (boost::ptr_list<Supervoxel>::iterator it = supervoxels.begin(); it != supervoxels.end(); )
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