#pragma once

#include <vector>

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/surface/processing.h>
#include <pcl/features/feature.h>

#include "Common.h"
#include "E57Data.h"
#include "BaseSegmenter.h"
#include "BaseReconstructor.h"

namespace RecRoom
{
	using E57ReconstructorBase = Reconstructor
		<
		pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointE57>, PointE57>,
		pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>, PointNDF>,
		pcl::Filter<PointE57xPCD>,
		pcl::FilterIndices<PointE57xPCD>,
		pcl::CloudSurfaceProcessing<PointE57xPCD, PointE57xPCD>,
		pcl::Feature<PointE57xPCD, PointE57xPCD>,
		pcl::Feature<PointE57xPCD, PointPCD>,
		Segmenter<PointPCD>,
		pcl::PointCloud<PointE57>,
		pcl::PointCloud<PointPCD>
		>;

	class E57Reconstructor : public E57ReconstructorBase
	{
	public:
		using Ptr = boost::shared_ptr<E57Reconstructor>;
		using ConstPtr = boost::shared_ptr<const E57Reconstructor>;

	public:
		// This constructor will create a new container
		E57Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution);

		// This constructor will load exist container
		E57Reconstructor(const boost::filesystem::path& filePath);

	public:
		virtual void FromJson(const nlohmann::json& j);
		virtual void ToJson(nlohmann::json& j) const;

		virtual void FromFile(const boost::filesystem::path& filePath);
		virtual void ToFile(const boost::filesystem::path& filePath) const { THROW_EXCEPTION("Interface is not implemented"); }

		virtual void ReconstructLOD(const double samplePercent = 0.125);
		virtual void ReconstructPointCloud() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructAlbedo() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSegment() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) { THROW_EXCEPTION("Interface is not implemented"); }

	protected:
		std::vector<E57ScanData::Ptr> scanMeta;
		boost::filesystem::path filePath;
	};
}

#include "E57Reconstructor.hpp"