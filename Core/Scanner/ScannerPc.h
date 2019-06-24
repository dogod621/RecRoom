#pragma once

#include "Common/Common.h"
#include "Common/Scan.h"
#include "Container/ContainerPcRAW.h"
#include "Preprocessor/PreprocessorPc.h"

namespace RecRoom
{
	class ScannerPc 
	{
	public:
		ScannerPc(
			const PTR(ContainerPcRAW)& containerPcRAW)
			: scanMeta(), containerPcRAW(containerPcRAW), preprocessor(nullptr)
		{
			if (!containerPcRAW)
				THROW_EXCEPTION("containerPcRAW is not set");
		}

	public:
		virtual void ShipData() const = 0;

	public:
		std::vector<ScanMeta> getScanMeta() const {return scanMeta;}
		ScanMeta getScanMeta(std::size_t i) const { return scanMeta[i]; }
		PTR(ContainerPcRAW) getContainerPcRAW () const { return containerPcRAW; }
		CONST_PTR(PreprocessorPc) getPreprocessor () const { return preprocessor; }

		void setPreprocessor(CONST_PTR(PreprocessorPc) preprocessor_) { preprocessor = preprocessor_; }

	protected:
		std::vector<ScanMeta> scanMeta;
		PTR(ContainerPcRAW) containerPcRAW;
		CONST_PTR(PreprocessorPc) preprocessor;
	};
}

#include "ScannerPc.hpp"