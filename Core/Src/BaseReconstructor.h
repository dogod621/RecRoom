#pragma once

#include "Common.h"
#include "Coordinate.h"
#include "BaseData.h"

namespace RecRoom
{
	template<
		class RAWContainer_, class NDFContainer_,
		class DownSampler_, class OutlierRemover_,
		class SurfaceProcesser_, class NormalEstimater_, class AlbedoEstimater_, class Segmenter_,
		class RawPointCloud_, class PointCloud_>
		class Reconstructor : public Data
	{
	public:
		using Ptr = boost::shared_ptr<Reconstructor< RAWContainer_, NDFContainer_, DownSampler_, OutlierRemover_, SurfaceProcesser_, NormalEstimater_, AlbedoEstimater_, Segmenter_, RawPointCloud_, PointCloud_>>;
		using ConstPtr = boost::shared_ptr<const Reconstructor< RAWContainer_, NDFContainer_, DownSampler_, OutlierRemover_, SurfaceProcesser_, NormalEstimater_, AlbedoEstimater_, Segmenter_, RawPointCloud_, PointCloud_>>;

		using RAWContainer = RAWContainer_;
		using NDFContainer = NDFContainer_;
		using DownSampler = DownSampler_;
		using OutlierRemover = OutlierRemover_;
		using SurfaceProcesser = SurfaceProcesser_;
		using NormalEstimater = NormalEstimater_;
		using AlbedoEstimater = AlbedoEstimater_;
		using Segmenter = Segmenter_;
		using RawPointCloud = RawPointCloud_;
		using PointCloud = PointCloud_;

		using RAWContainerPtr = boost::shared_ptr < RAWContainer_>;
		using NDFContainerPtr = boost::shared_ptr < NDFContainer_>;
		using DownSamplerPtr = boost::shared_ptr < DownSampler_>;
		using OutlierRemoverPtr = boost::shared_ptr < OutlierRemover_>;
		using SurfaceProcesserPtr = boost::shared_ptr < SurfaceProcesser_>;
		using NormalEstimaterPtr = boost::shared_ptr < NormalEstimater_>;
		using AlbedoEstimaterPtr = boost::shared_ptr < AlbedoEstimater_>;
		using SegmenterPtr = boost::shared_ptr < Segmenter_>;
		using RawPointCloudPtr = boost::shared_ptr < RawPointCloud_>;
		using PointCloudPtr = boost::shared_ptr < PointCloud_>;

		using RAWContainerConstPtr = boost::shared_ptr <const RAWContainer_>;
		using NDFContainerConstPtr = boost::shared_ptr <const NDFContainer_>;
		using DownSamplerConstPtr = boost::shared_ptr <const DownSampler_>;
		using OutlierRemoverConstPtr = boost::shared_ptr <const OutlierRemover_>;
		using SurfaceProcesserConstPtr = boost::shared_ptr <const SurfaceProcesser_>;
		using NormalEstimaterConstPtr = boost::shared_ptr <const NormalEstimater_>;
		using AlbedoEstimaterConstPtr = boost::shared_ptr <const AlbedoEstimater_>;
		using SegmenterConstPtr = boost::shared_ptr <const Segmenter_>;
		using RawPointCloudConstPtr = boost::shared_ptr <const RawPointCloud_>;
		using PointCloudConstPtr = boost::shared_ptr <const PointCloud_>;

	public:
		Reconstructor() : Data() {}

	public:
		virtual void ReconstructPointCloud() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructAlbedo() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSegment() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
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
		inline PointCloudPtr getPointCloud() const { return pointCloud; }

		inline void setDownSampler(DownSamplerPtr v) { downSampler = v; }
		inline void setOutlierRemover(OutlierRemoverPtr v) { outlierRemover = v; }
		inline void setSurfaceProcesser(SurfaceProcesserPtr v) { surfaceProcesser = v; }
		inline void setNormalEstimater(NormalEstimaterPtr v) { normalEstimater = v; }
		inline void setAlbedoEstimater(AlbedoEstimaterPtr v) { albedoEstimater = v; }
		inline void setSegmenter(SegmenterPtr v) { segmenter = v; }
		inline void setPointCloud(PointCloudPtr v) { pointCloud = v; }

	protected:
		RAWContainerPtr rawContainer;
		NDFContainerPtr ndfContainer;
		DownSamplerPtr downSampler;
		OutlierRemoverPtr outlierRemover;
		SurfaceProcesserPtr surfaceProcesser;
		NormalEstimaterPtr normalEstimater;
		AlbedoEstimaterPtr albedoEstimater;
		SegmenterPtr segmenter;
		PointCloudPtr pointCloud;
	};
}

#include "BaseReconstructor.hpp"