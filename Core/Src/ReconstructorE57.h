#pragma once

#include <vector>

#include "Common.h"
#include "DataE57.h"
#include "Reconstructor.h"

namespace RecRoom
{
#define MAX_NUM_SEGMENT 65534 // Althogh origin lable range is [0, 2^32-1) (2^32-1 is used for encode UNKNOWN). Due to the out of core mechanism, it will become [0, 2^16-1)

	//
	class ReconstructorE57 : public Reconstructor<PointE57, PointE57xPCD, PointPCD, DataScanE57>
	{
	public:
		using Self = ReconstructorE57;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		// This constructor will create a new container
		ReconstructorE57(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution);

		// This constructor will load exist container
		ReconstructorE57(const boost::filesystem::path& filePath);

	public:
		virtual void FromFile(const boost::filesystem::path& filePath);
		virtual void ToFile(const boost::filesystem::path& filePath) const { THROW_EXCEPTION("Interface is not implemented"); }

		virtual void ReconstructPointCloud();
		virtual void ReconstructAlbedo();
		virtual void ReconstructSegment();

		/*virtual void ReconstructNDF() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ReconstructSurcafe() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) { THROW_EXCEPTION("Interface is not implemented"); }*/
	};
}

#include "ReconstructorE57.hpp"