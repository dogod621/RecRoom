#pragma once

#include "E57Format.h"

#include "Common/StringUtils.h"
#include "ScannerPc.h"

namespace RecRoom
{
	class ScannerPcE57 : public ScannerPc, public AsyncAble
	{
	public:
		ScannerPcE57(
			const boost::filesystem::path& filePath,
			const PTR(ContainerPcRAW)& containerPcRAW);

		~ScannerPcE57()
		{
			if (imageFileE57)
				imageFileE57->close();
		}

	public:
		virtual void LoadPcRAW(int serialNumber, PcRAW& pc, bool local = false) const;
		virtual void LoadPcLF(int serialNumber, PcLF& pc, bool local = false) const { THROW_EXCEPTION("ScannerPcE57 cannot retrieve light field data") };

	protected:
		virtual void ShipPcRAW() const;
		virtual void ShipPcLF() const { THROW_EXCEPTION("ScannerPcE57 cannot retrieve light field data") };
		
	public:
		PTR(e57::ImageFile) getImageFileE57() const { return imageFileE57; }
		CONST_PTR(e57::VectorNode) getData3DE57() const { return data3DE57; }
		CONST_PTR(e57::VectorNode) getImages2DE57() const { return images2DE57; }

	protected:
		PTR(e57::ImageFile) imageFileE57;
		CONST_PTR(e57::VectorNode) data3DE57;
		CONST_PTR(e57::VectorNode) images2DE57;
		
		void LoadScanMetaSet();
	};

	//
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::Node& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::StructureNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::VectorNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::CompressedVectorNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::IntegerNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::ScaledIntegerNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::FloatNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::StringNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::BlobNode& pNode);
	std::ostream& operator << (std::ostream& os, const ScannerPcE57& v);
}

#include "ScannerPcE57.hpp"