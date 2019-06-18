#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/surface/processing.h>
#include <pcl/features/feature.h>

#include "Common.h"
#include "E57Data.h"
#include "BaseSegmenter.h"

namespace RecRoom
{
	class E57Reconstructor
	{
	public:
		using RAWContainer = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointE57>, PointE57>;
		using NDFContainer = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>, PointNDF>;
		using DownSampler = pcl::Filter<PointE57xPCD>;
		using OutlierRemover = pcl::FilterIndices<PointE57xPCD>;
		using SurfaceProcesser = pcl::CloudSurfaceProcessing<PointE57xPCD, PointE57xPCD>;
		using NormalEstimater = pcl::Feature<PointE57xPCD, PointE57xPCD>;
		using AlbedoEstimater = pcl::Feature<PointE57xPCD, PointPCD>;
		using Segmenter = Segmenter<PointPCD>;
		using PointCloud = pcl::PointCloud<PointPCD>;

	public:
		// This constructor will create a new container
		E57Reconstructor(const boost::filesystem::path& rawContainerPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution);

		// This constructor will load exist container
		E57Reconstructor(const boost::filesystem::path& rawContainerPath);

	public:
		virtual void LoadFile(const boost::filesystem::path& filePath, const double samplePercent) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructLOD(const double samplePercent = 0.125) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructPointCloud() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructAlbedo() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSegment() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) { THROW_EXCEPTION("Interface is not implemented"); }

	public:
		inline RAWContainer::Ptr getRAWContainer() const { return rawContainer; }
		inline NDFContainer::Ptr getNDFContainer() const { return ndfContainer; }
		inline DownSampler::Ptr getDownSampler() const { return downSampler; }
		inline OutlierRemover::Ptr getOutlierRemover() const { return outlierRemover; }
		inline SurfaceProcesser::Ptr getSurfaceProcesser() const { return surfaceProcesser; }
		inline NormalEstimater::Ptr getNormalEstimater() const { return normalEstimater; }
		inline AlbedoEstimater::Ptr getAlbedoEstimater() const { return albedoEstimater; }
		inline Segmenter::Ptr getSegmenter() const { return segmenter; }
		inline PointCloud::Ptr getPointCloud() const { return pointCloud; }

		inline void setDownSampler(DownSampler::Ptr v) { downSampler = v; }
		inline void setOutlierRemover(OutlierRemover::Ptr v) { outlierRemover = v; }
		inline void setSurfaceProcesser(SurfaceProcesser::Ptr v) { surfaceProcesser = v; }
		inline void setNormalEstimater(NormalEstimater::Ptr v) { normalEstimater = v; }
		inline void setAlbedoEstimater(AlbedoEstimater::Ptr v) { albedoEstimater = v; }
		inline void setSegmenter(Segmenter::Ptr v) { segmenter = v; }
		inline void setPointCloud(PointCloud::Ptr v) { pointCloud = v; }

	protected:
		boost::filesystem::path rawContainerPath;
		RAWContainer::Ptr rawContainer;
		NDFContainer::Ptr ndfContainer;
		DownSampler::Ptr downSampler;
		OutlierRemover::Ptr outlierRemover;
		SurfaceProcesser::Ptr surfaceProcesser;
		NormalEstimater::Ptr normalEstimater;
		AlbedoEstimater::Ptr albedoEstimater;
		Segmenter::Ptr segmenter;
		PointCloud::Ptr pointCloud;
	};
}

#include "E57Reconstructor.hpp"