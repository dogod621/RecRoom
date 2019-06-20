#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/surface/processing.h>
#include <pcl/features/feature.h>

#include "Common.h"
#include "Coordinate.h"
#include "Data.h"
#include "Segmenter.h"

namespace RecRoom
{
	template<class PointType>
	class Container : public Data, public pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>
	{
	public:
		using Self = Container<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

		Container(const boost::filesystem::path &root_node_name, const bool load_all)
			: Data(), pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>(root_node_name, load_all) {}
		Container(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const boost::filesystem::path &root_node_name, const std::string &coord_sys)
			: Data(), pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>(min, max, resolution_arg, root_node_name, coord_sys) {}
		Container(const boost::uint64_t max_depth, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::filesystem::path &root_node_name, const std::string &coord_sys)
			: Data(), pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>(max_depth, min, max, root_node_name, coord_sys) {}
	};

	template<class PointType>
	class DownSampler : public Data, public pcl::Filter<PointType>
	{
	public:
		using Self = DownSampler<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);
	};

	template<class PointType>
	class OutlierRemover : public Data, public pcl::FilterIndices<PointType>
	{
	public:
		using Self = OutlierRemover<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);
	};

	template<class PointTypeIn, class PointTypeOut>
	class SurfaceProcesser : public Data, public pcl::CloudSurfaceProcessing<PointTypeIn, PointTypeOut>
	{
	public:
		using Self = SurfaceProcesser<PointTypeIn, PointTypeOut>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);
	};

	template<class PointTypeIn, class PointTypeOut>
	class NormalEstimater : public Data, public pcl::Feature<PointTypeIn, PointTypeOut>
	{
	public:
		using Self = NormalEstimater<PointTypeIn, PointTypeOut>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);
	};

	template<class PointTypeIn, class PointTypeOut>
	class AlbedoEstimater : public Data, public pcl::Feature<PointTypeIn, PointTypeOut>
	{
	public:
		using Self = AlbedoEstimater<PointTypeIn, PointTypeOut>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);
	};

	enum ReconstructStatus : Flag
	{
		ReconstructStatus_UNKNOWN = 0,
		POINT_CLOUD = 1 << 1,
		ALBEDO = 1 << 2,
		SEGMENT = 1 << 3,
		NDF = 1 << 4,
		SURFACE = 1 << 5
	};

	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	class Reconstructor : public Data
	{
	public:
		using Self = Reconstructor<PointRaw, PointMed, PointRec, MetaScan>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);
		
		using PointRawT = PointRaw;
		using PointMedT = PointMed;
		using PointRecT = PointRec;
		using MetaScanT = MetaScan;

		using ContainerRawT = Container<PointRaw>;
		using ContainerNDFT = Container<PointNDF>;
		using PointCloudRawT = pcl::PointCloud<PointRaw>;
		using PointCloudMedT = pcl::PointCloud<PointMed>;
		using PointCloudRecT = pcl::PointCloud<PointRec>;
		using MetaScansT = std::vector<PTR(MetaScan)>;

		using DownSamplerT = DownSampler<PointRaw>;
		using OutlierRemoverT = OutlierRemover<PointMed>;
		using SurfaceProcesserT = SurfaceProcesser<PointMed, PointMed>;
		using NormalEstimaterT = NormalEstimater<PointMed, PointMed>;
		using AlbedoEstimaterT = AlbedoEstimater<PointMed, PointMed>;
		using SegmenterT = Segmenter<PointRec>;

	public:
		// This constructor will create a new container
		Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const double outofCoreLeafOverlap = -1);

		// This constructor will load exist container
		Reconstructor(const boost::filesystem::path& filePath);

		void ReconstructLOD(const double samplePercent = 0.125);

	public:
		virtual void FromJson(const nlohmann::json& j);
		virtual void ToJson(nlohmann::json& j) const;

		virtual void ReconstructPointCloud() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructAlbedo() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSegment() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSurcafe() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) { THROW_EXCEPTION("Interface is not implemented"); }

	public:
		inline PTR(ContainerRawT) getContainerRaw() { return containerRaw; }
		inline PTR(ContainerNDFT) getContainerNDF() { return containerNDF; }
		inline PTR(PointCloudRecT) getPointCloudRec() { return pointCloudRec; }

		inline PTR(DownSamplerT) getDownSampler() { return downSampler; }
		inline PTR(OutlierRemoverT) getOutlierRemover() { return outlierRemover; }
		inline PTR(SurfaceProcesserT) getSurfaceProcesser() { return surfaceProcesser; }
		inline PTR(NormalEstimaterT) getNormalEstimater() { return normalEstimater; }
		inline PTR(AlbedoEstimaterT) getAlbedoEstimater() { return albedoEstimater; }
		inline PTR(SegmenterT) getSegmenter() { return segmenter; }
		inline PTR(MetaScansT) getMetaScans() { return metaScans; }

		inline CONST_PTR(ContainerRawT) getContainerRaw() const { return containerRaw; }
		inline CONST_PTR(ContainerNDFT) getContainerNDF() const { return containerNDF; }
		inline CONST_PTR(PointCloudRecT) getPointCloudRec() const { return pointCloudRec; }

		inline CONST_PTR(DownSamplerT) getDownSampler() const { return downSampler; }
		inline CONST_PTR(OutlierRemoverT) getOutlierRemover() const { return outlierRemover; }
		inline CONST_PTR(SurfaceProcesserT) getSurfaceProcesser() const { return surfaceProcesser; }
		inline CONST_PTR(NormalEstimaterT) getNormalEstimater() const { return normalEstimater; }
		inline CONST_PTR(AlbedoEstimaterT) getAlbedoEstimater() const { return albedoEstimater; }
		inline CONST_PTR(SegmenterT) getSegmenter() const { return segmenter; }
		inline CONST_PTR(MetaScansT) getMetaScans() const { return metaScans; }

		inline void setDownSampler(PTR(DownSamplerT) v) { downSampler = v; }
		inline void setOutlierRemover(PTR(OutlierRemoverT) v) { outlierRemover = v; }
		inline void setSurfaceProcesser(PTR(SurfaceProcesserT) v) { surfaceProcesser = v; }
		inline void setNormalEstimater(PTR(NormalEstimaterT) v) { normalEstimater = v; }
		inline void setAlbedoEstimater(PTR(AlbedoEstimaterT) v) { albedoEstimater = v; }
		inline void setSegmenter(PTR(SegmenterT) v) { segmenter = v; }

	protected:
		boost::filesystem::path filePath;
		PTR(MetaScansT) metaScans;
		ReconstructStatus status;
		double outofCoreLeafOverlap;

		PTR(ContainerRawT) containerRaw;
		PTR(ContainerNDFT) containerNDF;
		PTR(PointCloudRecT) pointCloudRec;

		PTR(DownSamplerT) downSampler;
		PTR(OutlierRemoverT) outlierRemover;
		PTR(SurfaceProcesserT) surfaceProcesser;
		PTR(NormalEstimaterT) normalEstimater;
		PTR(AlbedoEstimaterT) albedoEstimater;
		PTR(SegmenterT) segmenter;
	};
}

#include "Reconstructor.hpp"