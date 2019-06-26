#include "Common.h"

namespace RecRoom
{
	std::mutex Common::gLock;
	double Common::eps = std::numeric_limits<float>::epsilon();
	Eigen::Vector3d Common::tempVec1 = Eigen::Vector3d(1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0));
	Eigen::Vector3d Common::tempVec2 = Eigen::Vector3d(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0.0);
	Eigen::Vector3d Common::tempVec3 = Eigen::Vector3d(1.0 / std::sqrt(2.0), 0.0, 1.0 / std::sqrt(2.0));
	Eigen::Vector3d Common::tempVec4 = Eigen::Vector3d(0.0, 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0));

	bool Common::GenFrame(const Eigen::Vector3d& notmal, Eigen::Vector3d& tangent, Eigen::Vector3d& bitangent)
	{
		tangent = notmal.cross(Common::tempVec1);
		double tangentNorm = tangent.norm();
		if (tangentNorm > Common::eps)
		{
			tangent /= tangentNorm;
			bitangent = notmal.cross(tangent);
			bitangent /= bitangent.norm();
			return true;
		}
		else
		{
			tangent = notmal.cross(Common::tempVec2);
			tangentNorm = tangent.norm();
			if (tangentNorm > Common::eps)
			{
				tangent /= tangentNorm;
				bitangent = notmal.cross(tangent);
				bitangent /= bitangent.norm();
				return true;
			}
			else
			{
				tangent = notmal.cross(Common::tempVec3);
				tangentNorm = tangent.norm();
				if (tangentNorm > Common::eps)
				{
					tangent /= tangentNorm;
					bitangent = notmal.cross(tangent);
					bitangent /= bitangent.norm();
					return true;
				}
				else
				{
					tangent = notmal.cross(Common::tempVec4);
					tangentNorm = tangent.norm();
					if (tangentNorm > Common::eps)
					{
						tangent /= tangentNorm;
						bitangent = notmal.cross(tangent);
						bitangent /= bitangent.norm();
						return true;
					}
				}
			}
		}
		return false;
	}

	bool Common::IsUnitVector(const Eigen::Vector3d& v)
	{
		return std::abs(v.norm() - 1.0f) < 0.001;
	}

	DumpAble::DumpAble(const std::string& className, const boost::filesystem::path& filePath)
		: className(className), filePath(filePath)
	{
		if (!boost::filesystem::exists(filePath))
		{
			boost::filesystem::create_directory(filePath);
			PRINT_INFO("Create directory: " + filePath.string());
		}
	}

	void DumpAble::Load()
	{
		std::string path = (filePath / boost::filesystem::path(className)).string();
		std::ifstream file(path, std::ios_base::in);
		if (!file)
			THROW_EXCEPTION("Load file " + path + " failed.");

		nlohmann::json j;
		file >> j;

		this->Load(j);
	}

	void DumpAble::Dump() const
	{
		std::string path = (filePath / boost::filesystem::path(className)).string();
		std::ofstream file(path, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + path + " failed.");

		nlohmann::json j;

		this->Dump(j);

		file << j;
		file.close();
	}

	bool DumpAble::CheckExist() const
	{
		if (!boost::filesystem::is_directory(filePath))
			return false;
		if (!boost::filesystem::exists(filePath / boost::filesystem::path(className)))
			return false;
		return true;
	}
}