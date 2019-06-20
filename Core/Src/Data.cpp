#include "Data.h"

namespace RecRoom
{
	void Data3D::FromJson(const nlohmann::json& j)
	{ 
		{
			nlohmann::json::const_iterator it = j["transform"].begin();
			for (int r = 0; r < 4; ++r)
			{
				for (int c = 0; c < 4; ++c)
				{
					transform(r, c) = *it;
					++it;
				}
			}
		}

		{
			nlohmann::json::const_iterator it = j["position"].begin();
			position.x() = *it; ++it;
			position.y() = *it; ++it;
			position.z() = *it; ++it;
		}

		{
			nlohmann::json::const_iterator it = j["orientation"].begin();
			orientation.x() = *it; ++it;
			orientation.y() = *it; ++it;
			orientation.z() = *it; ++it;
			orientation.w() = *it; ++it;
		}
	}

	void Data3D::ToJson(nlohmann::json& j) const
	{
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				j["transform"].push_back(transform(r, c));

		j["position"].push_back(position.x());
		j["position"].push_back(position.y());
		j["position"].push_back(position.z());

		j["orientation"].push_back(orientation.x());
		j["orientation"].push_back(orientation.y());
		j["orientation"].push_back(orientation.z());
		j["orientation"].push_back(orientation.w());
	}
}