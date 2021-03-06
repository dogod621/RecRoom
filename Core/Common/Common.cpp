#include <thread>

#include "Common.h"

namespace RecRoom
{
	std::mutex Common::gLock;

	Eigen::Vector3d Common::tempVec1_d = Eigen::Vector3d(1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0));
	Eigen::Vector3d Common::tempVec2_d = Eigen::Vector3d(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0.0);
	Eigen::Vector3d Common::tempVec3_d = Eigen::Vector3d(1.0 / std::sqrt(2.0), 0.0, 1.0 / std::sqrt(2.0));
	Eigen::Vector3d Common::tempVec4_d = Eigen::Vector3d(0.0, 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0));

	Eigen::Vector3f Common::tempVec1_f = Eigen::Vector3f(1.0f / std::sqrt(3.0f), 1.0f / std::sqrt(3.0f), 1.0f / std::sqrt(3.0f));
	Eigen::Vector3f Common::tempVec2_f = Eigen::Vector3f(1.0f / std::sqrt(2.0f), 1.0f / std::sqrt(2.0f), 0.0f);
	Eigen::Vector3f Common::tempVec3_f = Eigen::Vector3f(1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f));
	Eigen::Vector3f Common::tempVec4_f = Eigen::Vector3f(0.0f, 1.0f / std::sqrt(2.0f), 1.0f / std::sqrt(2.0f));

	bool Common::GenFrame(const Eigen::Vector3d& notmal, Eigen::Vector3d& tangent, Eigen::Vector3d& bitangent)
	{
		tangent = notmal.cross(Common::tempVec1_d);
		double tangentNorm = tangent.norm();
		if (tangentNorm > std::numeric_limits<float>::epsilon())
		{
			tangent /= tangentNorm;
			bitangent = notmal.cross(tangent);
			bitangent /= bitangent.norm();
			return true;
		}
		else
		{
			tangent = notmal.cross(Common::tempVec2_d);
			tangentNorm = tangent.norm();
			if (tangentNorm > std::numeric_limits<float>::epsilon())
			{
				tangent /= tangentNorm;
				bitangent = notmal.cross(tangent);
				bitangent /= bitangent.norm();
				return true;
			}
			else
			{
				tangent = notmal.cross(Common::tempVec3_d);
				tangentNorm = tangent.norm();
				if (tangentNorm > std::numeric_limits<float>::epsilon())
				{
					tangent /= tangentNorm;
					bitangent = notmal.cross(tangent);
					bitangent /= bitangent.norm();
					return true;
				}
				else
				{
					tangent = notmal.cross(Common::tempVec4_d);
					tangentNorm = tangent.norm();
					if (tangentNorm > std::numeric_limits<float>::epsilon())
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

	bool Common::GenFrame(const Eigen::Vector3f& notmal, Eigen::Vector3f& tangent, Eigen::Vector3f& bitangent)
	{
		tangent = notmal.cross(Common::tempVec1_f);
		float tangentNorm = tangent.norm();
		if (tangentNorm > std::numeric_limits<float>::epsilon())
		{
			tangent /= tangentNorm;
			bitangent = notmal.cross(tangent);
			bitangent /= bitangent.norm();
			return true;
		}
		else
		{
			tangent = notmal.cross(Common::tempVec2_f);
			tangentNorm = tangent.norm();
			if (tangentNorm > std::numeric_limits<float>::epsilon())
			{
				tangent /= tangentNorm;
				bitangent = notmal.cross(tangent);
				bitangent /= bitangent.norm();
				return true;
			}
			else
			{
				tangent = notmal.cross(Common::tempVec3_f);
				tangentNorm = tangent.norm();
				if (tangentNorm > std::numeric_limits<float>::epsilon())
				{
					tangent /= tangentNorm;
					bitangent = notmal.cross(tangent);
					bitangent /= bitangent.norm();
					return true;
				}
				else
				{
					tangent = notmal.cross(Common::tempVec4_f);
					tangentNorm = tangent.norm();
					if (tangentNorm > std::numeric_limits<float>::epsilon())
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

	void ThreadAble::SetNumberOfThreads(unsigned int numThreads_)
	{
		if (numThreads_ == 0)
#ifdef _OPENMP
			numThreads = omp_get_num_procs();
#else
			numThreads = std::thread::hardware_concurrency();
#endif
		else
			numThreads = numThreads_;
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

		Load(j);
	}

	void DumpAble::Dump() const
	{
		std::string path = (filePath / boost::filesystem::path(className)).string();
		std::ofstream file(path, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + path + " failed.");

		nlohmann::json j;

		Dump(j);

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