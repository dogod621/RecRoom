#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	template<class PointType>
	class MesherPcPoisson : public MesherPc<PointType>
	{
	public:
		using Sampler = MesherPc<PointType>::Sampler;
		using Filter = MesherPc<PointType>::Filter;
		using Interpolator = MesherPc<PointType>::Interpolator;
		using InterpolatorNearest = MesherPc<PointType>::InterpolatorNearest;

	public:
		MesherPcPoisson(
			int depth = 8,
			int minDepth = 5,
			float pointWeight = 4.0f,
			float scale = 11.f,
			int solverDivide = 8,
			int isoDivide = 8,
			float samplesPerNode = 1.0f,
			bool confidence = false,
			bool outputPolygons = false,
			bool manifold = true,
			int degree = 2,
			CONST_PTR(Sampler) preprocessSampler = nullptr,
			CONST_PTR(Filter) preprocessFilter = nullptr,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: depth(depth), minDepth(minDepth), pointWeight(pointWeight), scale(scale), solverDivide(solverDivide),
			isoDivide(isoDivide), samplesPerNode(samplesPerNode), confidence(confidence), outputPolygons(outputPolygons),
			manifold(manifold), degree(degree), 
			MesherPc<PointType>(preprocessSampler, preprocessFilter, fieldInterpolator) 
		{
			name = "MesherPcPoisson";
		}

	protected:
		virtual void ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const;

	public:
		int getDepth() const { return depth; }
		int getMinDepth() const { return minDepth; }
		float getPointWeight() const { return pointWeight; }
		float getScale() const { return scale; }
		int getSolverDivide() const { return solverDivide; }
		int getIsoDivide() const { return isoDivide; }
		float getSamplesPerNode() const { return samplesPerNode; }
		bool getConfidence() const { return confidence; }
		bool getOutputPolygons() const { return outputPolygons; }
		bool getManifold() const { return manifold; }
		int getDegree() const { return degree; }

		void setDepth(int v) { depth = v; }
		void setMinDepth(int v) { minDepth = v; }
		void setPointWeight(float v) { pointWeight = v; }
		void setScale(float v) { scale = v; }
		void setSolverDivide(int v) { solverDivide = v; }
		void setIsoDivide(int v) { isoDivide = v; }
		void setSamplesPerNode(float v) { samplesPerNode = v; }
		void setConfidence(bool v) { confidence = v; }
		void setOutputPolygons(bool v) { outputPolygons = v; }
		void setManifold(bool v) { manifold = v; }
		void setDegree(int v) { degree = v; }

	protected:
		int depth;
		int minDepth;
		float pointWeight;
		float scale;
		int solverDivide;
		int isoDivide;
		float samplesPerNode;
		bool confidence;
		bool outputPolygons;
		bool manifold;
		int degree;
	};
}

#include "MesherPcPoisson.hpp"