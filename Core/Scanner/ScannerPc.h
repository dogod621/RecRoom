#pragma once

#include "Common/Common.h"
#include "Common/Scan.h"
#include "Container/ContainerPcRAW.h"
#include "Container/ContainerPcLF.h"
#include "Preprocessor/PreprocessorPc.h"

namespace RecRoom
{
	class ScannerPc 
	{
	public:
		ScannerPc(
			const PTR(ContainerPcRAW)& containerPcRAW,
			Scanner scanner)
			: scanner(scanner), scanMetaSet(), containerPcRAW(containerPcRAW), preprocessor(nullptr)
		{
			if (!containerPcRAW)
				THROW_EXCEPTION("containerPcRAW is not set");
		}

	public:
		virtual void ShipPcRAWData() const { THROW_EXCEPTION("Interface is not implemented") };
		virtual void ShipPcLFData() const { THROW_EXCEPTION("Interface is not implemented") };
		virtual void ShipPcRAWData(std::size_t asyncSize) const {THROW_EXCEPTION("Interface is not implemented")};
		virtual void ShipPcLFData(std::size_t asyncSize) const { THROW_EXCEPTION("Interface is not implemented") };

		virtual bool Valid(const PointRAW& pointRAW) const
		{
			return pcl::isFinite(pointRAW);
		}

	public:
		Scanner getScanner() const { return scanner; }
		std::vector<ScanMeta> getScanMetaSet() const {return scanMetaSet;}
		ScanMeta getScanMeta(std::size_t i) const { return scanMetaSet[i]; }
		PTR(ContainerPcRAW) getContainerPcRAW () const { return containerPcRAW; }
		PTR(ContainerPcLF) getContainerPcLF() const { return containerPcLF; }
		CONST_PTR(PreprocessorPc) getPreprocessor () const { return preprocessor; }

		void setPreprocessor(PTR(ContainerPcLF) containerPcLF_) { containerPcLF = containerPcLF_; }
		void setPreprocessor(CONST_PTR(PreprocessorPc) preprocessor_) { preprocessor = preprocessor_; }

	protected:
		Scanner scanner;
		std::vector<ScanMeta> scanMetaSet;
		PTR(ContainerPcRAW) containerPcRAW;
		PTR(ContainerPcLF) containerPcLF;
		CONST_PTR(PreprocessorPc) preprocessor;
	};
}

#include "ScannerPc.hpp"