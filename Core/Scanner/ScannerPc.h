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
			: scanner(scanner), scanMetaSet(new std::vector<ScanMeta>), containerPcRAW(containerPcRAW), preprocessor(nullptr)
		{
			if (!containerPcRAW)
				THROW_EXCEPTION("containerPcRAW is not set");
		}

	public:
		virtual void ShipPcRAW() const = 0;
		virtual void ShipPcLF() const = 0;
		virtual void LoadPcRAW(int serialNumber, PcRAW& pc, bool local = false) const = 0;
		virtual void LoadPcLF(int serialNumber, PcLF& pc, bool local = false) const = 0;

		virtual bool Valid(const PointRAW& pointRAW) const
		{
			return pcl::isFinite(pointRAW);
		}

	public:
		Scanner getScanner() const { return scanner; }
		PTR(std::vector<ScanMeta>) getScanMetaSet() const {return scanMetaSet;}

		const ScanMeta& getScanMeta(int serialNumber) const
		{ 
			if((serialNumber >= scanMetaSet->size()) || (serialNumber < 0))
				THROW_EXCEPTION("serialNumber is not valid");
			const ScanMeta& s = (*scanMetaSet)[serialNumber];
			if (s.serialNumber < 0)
				THROW_EXCEPTION("The scan is not valid");
			if (serialNumber != s.serialNumber)
				THROW_EXCEPTION("serialNumber is not match");
			return s; 
		}

		PTR(ContainerPcRAW) getContainerPcRAW () const { return containerPcRAW; }
		PTR(ContainerPcLF) getContainerPcLF() const { return containerPcLF; }
		CONST_PTR(PreprocessorPc) getPreprocessor () const { return preprocessor; }

		void setPreprocessor(PTR(ContainerPcLF) containerPcLF_) { containerPcLF = containerPcLF_; }
		void setPreprocessor(CONST_PTR(PreprocessorPc) preprocessor_) { preprocessor = preprocessor_; }

	protected:
		Scanner scanner;
		PTR(std::vector<ScanMeta>) scanMetaSet;
		PTR(ContainerPcRAW) containerPcRAW;
		PTR(ContainerPcLF) containerPcLF;
		CONST_PTR(PreprocessorPc) preprocessor;
	};
}

#include "ScannerPc.hpp"