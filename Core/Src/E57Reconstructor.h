#pragma once

#include <vector>

#include "Common.h"
#include "E57Data.h"
#include "BaseReconstructor.h"

namespace RecRoom
{
#define MAX_NUM_SEGMENT 65534 // Althogh origin lable range is [0, 2^32-1) (2^32-1 is used for encode UNKNOWN). Due to the out of core mechanism, it will become [0, 2^16-1)

	class E57Reconstructor : public Reconstructor<PointE57, PointE57xPCD, PointPCD>
	{
	public:
		using Ptr = boost::shared_ptr<E57Reconstructor>;
		using ConstPtr = boost::shared_ptr<const E57Reconstructor>;
		using Base = Reconstructor<PointE57, PointE57xPCD, PointPCD>;

	public:
		// This constructor will create a new container
		E57Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const double outofCoreLeafOverlap = -1);

		// This constructor will load exist container
		E57Reconstructor(const boost::filesystem::path& filePath, const double outofCoreLeafOverlap = -1);

	public:
		virtual void FromJson(const nlohmann::json& j);
		virtual void ToJson(nlohmann::json& j) const;

		virtual void FromFile(const boost::filesystem::path& filePath);
		virtual void ToFile(const boost::filesystem::path& filePath) const { THROW_EXCEPTION("Interface is not implemented"); }

		virtual void ReconstructPointCloud();
		virtual void ReconstructAlbedo() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSegment() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSurcafe() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) { THROW_EXCEPTION("Interface is not implemented"); }

	protected:
		std::vector<E57ScanData::Ptr> scanMeta;
	};
}

#include "E57Reconstructor.hpp"