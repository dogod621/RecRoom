#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	class MesherPcGP3 : public MesherPc
	{
	public:
		MesherPcGP3(
			double searchRadius,
			double mu,
			int maxNumNei = 100,
			double minAngle = M_PI / 18.0,
			double maxAngle = 2.0 * M_PI / 3.0,
			double epsAngle = M_PI / 4.0,
			bool consistent = false,
			bool consistentOrdering = false)
			: searchRadius(searchRadius), mu(mu), maxNumNei(maxNumNei), 
			minAngle(minAngle), maxAngle(maxAngle), epsAngle(epsAngle), 
			consistent(consistent), consistentOrdering(consistentOrdering), MesherPc() {}

	protected:
		virtual void ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const;

	public:
		double getSearchRadius() const { return searchRadius; }
		double getMu() const { return mu; }
		int getMaxNumNei() const { return maxNumNei; }
		double getMinAngle() const { return minAngle; }
		double getMaxAngle() const { return maxAngle; }
		double getEpsAngle() const { return epsAngle; }
		bool getConsistent() const { return consistent; }
		bool getConsistentOrdering() const { return consistentOrdering; }

		void setSearchRadius(double v) { searchRadius = v; }
		void setMu(double v) { mu = v; }
		void setMaxNumNei(int v) { maxNumNei = v; }
		void setMinAngle(double v) { minAngle = v; }
		void setMaxAngle(double v) { maxAngle = v; }
		void setEpsAngle(double v) { epsAngle = v; }
		void setConsistent(bool v) { consistent = v; }
		void setConsistentOrdering(bool v) { consistentOrdering = v; }

	protected:
		// brief The nearest neighbors search radius for each point and the maximum edge length.
		double searchRadius;
		
		// brief The nearest neighbor distance multiplier to obtain the final search radius.
		double mu;

		// brief The maximum number of nearest neighbors accepted by searching.
		int maxNumNei;

		// brief The preferred minimum angle for the triangles.
		double minAngle;

		// brief The maximum angle for the triangles.
		double maxAngle;

		// brief Maximum surface angle.
		double epsAngle;

		// brief Set this to true if the normals of the input are consistently oriented.
		bool consistent;

		// brief Set this to true if the output triangle vertices should be consistently oriented.
		bool consistentOrdering;
	};
}

#include "MesherPcGP3.hpp"