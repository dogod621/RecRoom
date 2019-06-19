#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/surface/processing.h>
#include <pcl/features/feature.h>

#include "Common.h"
#include "Coordinate.h"
#include "BaseData.h"
#include "BaseSegmenter.h"

namespace RecRoom
{
	template<class RawPointType_, class MedPointType_, class RecPointType_>
	class Reconstructor : public Data
	{
	public:
		using Base = Data;
		using Self = Reconstructor<RawPointType_, MedPointType_, RecPointType_>;
		using Ptr = boost::shared_ptr<Self>;
		using ConstPtr = boost::shared_ptr<Self>;
		
		using RawPointType = RawPointType_;
		using MedPointType = MedPointType_;
		using RecPointType = RecPointType_;

		using RAWContainer = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<RawPointType>, RawPointType>;
		using NDFContainer = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>, PointNDF>;
		using DownSampler = pcl::Filter<RawPointType>;
		using OutlierRemover = pcl::FilterIndices<MedPointType>;
		using SurfaceProcesser = pcl::CloudSurfaceProcessing<MedPointType, MedPointType>;
		using NormalEstimater = pcl::Feature<MedPointType, MedPointType>;
		using AlbedoEstimater = pcl::Feature<MedPointType, MedPointType>;
		using Segmenter = Segmenter<MedPointType>;
		using RawPointCloud = pcl::PointCloud<RawPointType>;
		using MedPointCloud = pcl::PointCloud<MedPointType>;
		using RecPointCloud = pcl::PointCloud<RecPointType>;

		using RAWContainerPtr = boost::shared_ptr<RAWContainer>;
		using NDFContainerPtr = boost::shared_ptr<NDFContainer>;
		using DownSamplerPtr = boost::shared_ptr<DownSampler>;
		using OutlierRemoverPtr = boost::shared_ptr<OutlierRemover>;
		using SurfaceProcesserPtr = boost::shared_ptr<SurfaceProcesser>;
		using NormalEstimaterPtr = boost::shared_ptr<NormalEstimater>;
		using AlbedoEstimaterPtr = boost::shared_ptr<AlbedoEstimater>;
		using SegmenterPtr = boost::shared_ptr<Segmenter>;
		using RawPointCloudPtr = boost::shared_ptr<RawPointCloud>;
		using MedPointCloudPtr = boost::shared_ptr<MedPointCloud>;
		using RecPointCloudPtr = boost::shared_ptr<RecPointCloud>;

		using RAWContainerConstPtr = boost::shared_ptr<const RAWContainer>;
		using NDFContainerConstPtr = boost::shared_ptr<const NDFContainer>;
		using DownSamplerConstPtr = boost::shared_ptr<const DownSampler>;
		using OutlierRemoverConstPtr = boost::shared_ptr<const OutlierRemover>;
		using SurfaceProcesserConstPtr = boost::shared_ptr<const SurfaceProcesser>;
		using NormalEstimaterConstPtr = boost::shared_ptr<const NormalEstimater>;
		using AlbedoEstimaterConstPtr = boost::shared_ptr<const AlbedoEstimater>;
		using SegmenterConstPtr = boost::shared_ptr<const Segmenter>;
		using RawPointCloudConstPtr = boost::shared_ptr<const RawPointCloud>;
		using MedPointCloudConstPtr = boost::shared_ptr<const MedPointCloud>;
		using RecPointCloudConstPtr = boost::shared_ptr<const RecPointCloud>;

	public:
		// This constructor will create a new container
		Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const double outofCoreLeafOverlap = -1);

		// This constructor will load exist container
		Reconstructor(const boost::filesystem::path& filePath, const double outofCoreLeafOverlap = -1);

		void ReconstructLOD(const double samplePercent = 0.125);

	public:
		virtual void ReconstructPointCloud() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructAlbedo() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSegment() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSurcafe() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) { THROW_EXCEPTION("Interface is not implemented"); }

	public:
		inline RAWContainerPtr getRAWContainer() const { return rawContainer; }
		inline NDFContainerPtr getNDFContainer() const { return ndfContainer; }
		inline DownSamplerPtr getDownSampler() const { return downSampler; }
		inline OutlierRemoverPtr getOutlierRemover() const { return outlierRemover; }
		inline SurfaceProcesserPtr getSurfaceProcesser() const { return surfaceProcesser; }
		inline NormalEstimaterPtr getNormalEstimater() const { return normalEstimater; }
		inline AlbedoEstimaterPtr getAlbedoEstimater() const { return albedoEstimater; }
		inline SegmenterPtr getSegmenter() const { return segmenter; }
		inline RecPointCloudPtr getPointCloud() const { return recPointCloud; }

		inline void setDownSampler(DownSamplerPtr v) { downSampler = v; }
		inline void setOutlierRemover(OutlierRemoverPtr v) { outlierRemover = v; }
		inline void setSurfaceProcesser(SurfaceProcesserPtr v) { surfaceProcesser = v; }
		inline void setNormalEstimater(NormalEstimaterPtr v) { normalEstimater = v; }
		inline void setAlbedoEstimater(AlbedoEstimaterPtr v) { albedoEstimater = v; }
		inline void setSegmenter(SegmenterPtr v) { segmenter = v; }
		inline void setRecPointCloud(RecPointCloudPtr v) { recPointCloud = v; }

	protected:
		RAWContainerPtr rawContainer;
		NDFContainerPtr ndfContainer;
		DownSamplerPtr downSampler;
		OutlierRemoverPtr outlierRemover;
		SurfaceProcesserPtr surfaceProcesser;
		NormalEstimaterPtr normalEstimater;
		AlbedoEstimaterPtr albedoEstimater;
		SegmenterPtr segmenter;
		RecPointCloudPtr recPointCloud;

		boost::filesystem::path filePath;
		double outofCoreLeafOverlap;
	};
}

#include "BaseReconstructor.hpp"