#include <iomanip>

#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "nlohmann/json.hpp"

#include "Common/AsyncProcess.h"

#include "ScannerPcBLK360.h"

namespace RecRoom
{
	//
	ScannerPcBLK360::ScannerPcBLK360(
		const boost::filesystem::path& filePath_,
		const boost::filesystem::path& e57FilePath,
		const boost::filesystem::path& lfFilePath,
		const PTR(ContainerPcRAW)& containerPcRAW,
		unsigned char colorThresh)
		: DumpAble("ScannerPcBLK360", filePath_), ScannerPcE57(e57FilePath, containerPcRAW), colorThresh(colorThresh)
	{
		if (CheckExist())
		{
			Load();
		}
		else
		{
			Dump();
		}
	}

	bool ScannerPcBLK360::ToScanLaser(const PointRAW& scanPoint, ScanLaser& scanLaser) const
	{
#ifdef INPUT_PERPOINT_SERIAL_NUMBER
		if (!scanPoint.HasSerialNumber())
		{
			PRINT_WARNING("!scanPoint.HasSerialNumber(), ignore");
			return false;
		}

		const ScanMeta& scanMeta = getScanMeta(scanPoint.serialNumber);

#		ifdef INPUT_PERPOINT_INTENSITY

		scanLaser.hitPosition = Eigen::Vector3d(scanPoint.x, scanPoint.y, scanPoint.z);
		scanLaser.incidentDirection = scanMeta.position - scanLaser.hitPosition;
		scanLaser.hitDistance = scanLaser.incidentDirection.norm();
		scanLaser.incidentDirection /= scanLaser.hitDistance;
		scanLaser.reflectedDirection = scanLaser.incidentDirection;

		// Ref - BLK 360 Spec - laser wavelength & Beam divergence : https://lasers.leica-geosystems.com/global/sites/lasers.leica-geosystems.com.global/files/leica_media/product_documents/blk/853811_leica_blk360_um_v2.0.0_en.pdf
		// Ref - Gaussian beam : https://en.wikipedia.org/wiki/Gaussian_beam
		// Ref - Beam divergence to Beam waist(w0) : http://www2.nsysu.edu.tw/optics/laser/angle.htm
		double temp = scanLaser.hitDistance / 26.2854504782;
		scanLaser.beamFalloff = 1.0f / (1 + temp * temp);
		scanLaser.intensity = (double)scanPoint.intensity;
		return true;

#		else
		PRINT_WARNING("INPUT_PERPOINT_INTENSITY is not definded");
		return false;
#		endif
#else
		PRINT_WARNING("INPUT_PERPOINT_SERIAL_NUMBER is not definded");
		return false;
#endif
	}

	void ScannerPcBLK360::Load(const nlohmann::json& j)
	{
		if (j.find("colorThresh") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"colorThresh\"");
		colorThresh = j["colorThresh"];
	}

	void ScannerPcBLK360::Dump(nlohmann::json& j) const
	{
		j["colorThresh"] = colorThresh;
	}
}