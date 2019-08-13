#include <fstream>

#include "PCLUtils.h"

// 
namespace RecRoom
{
	/*union LabelUnion
	{
		uint32_t uint32;
		int32_t int32;

		LabelUnion() {}
	};*/

	union ColorUnion
	{
		pcl::RGB rgba;
		uint32_t uint32;

		ColorUnion() {}
	};

	bool Copy(std::size_t numPoints, std::size_t pointSize, const std::vector<pcl::uint8_t>& data,
		const pcl::PCLPointField& field, uint8_t datatype, void* vector_)
	{
		if (field.datatype != datatype)
		{
			PRINT_ERROR("field.datatype is not support");
			return false;
		}

		switch (datatype)
		{
		case pcl::PCLPointField::FLOAT32:
		{
			std::vector<float>& vector = *(std::vector<float>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(float));
			}
		} break;

		case pcl::PCLPointField::FLOAT64:
		{
			std::vector<double>& vector = *(std::vector<double>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(double));
			}
		} break;

		case pcl::PCLPointField::INT8:
		{
			std::vector<int8_t>& vector = *(std::vector<int8_t>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(int8_t));
			}
		} break;

		case pcl::PCLPointField::INT16:
		{
			std::vector<int16_t>& vector = *(std::vector<int16_t>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(int16_t));
			}
		} break;

		case pcl::PCLPointField::INT32:
		{
			std::vector<int32_t>& vector = *(std::vector<int32_t>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(int32_t));
			}
		} break;

		case pcl::PCLPointField::UINT8:
		{
			std::vector<uint8_t>& vector = *(std::vector<uint8_t>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(uint8_t));
			}
		} break;

		case pcl::PCLPointField::UINT16:
		{
			std::vector<uint16_t>& vector = *(std::vector<uint16_t>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(uint16_t));
			}
		} break;

		case pcl::PCLPointField::UINT32:
		{
			std::vector<uint32_t>& vector = *(std::vector<uint32_t>*)vector_;
			vector.resize(numPoints);
			for (std::size_t i = 0; i < vector.size(); ++i)
			{
				std::memcpy(&vector[i], &data[i * pointSize + field.offset], sizeof(uint32_t));
			}
		} break;

		default:
			PRINT_ERROR("datatype is not support");
			return false;
		}
		return true;
	}

	int SaveAsPLY(const std::string &fileName, const Mesh &mesh, unsigned precision, bool binary)
	{
		if (mesh.cloud.data.empty())
		{
			PRINT_ERROR("Input mesh has no data");
			return -1;
		}

		//
		int x_index = getFieldIndex(mesh.cloud, "x");
		int y_index = getFieldIndex(mesh.cloud, "y");
		int z_index = getFieldIndex(mesh.cloud, "z");
		int normal_x_index = getFieldIndex(mesh.cloud, "normal_x");
		int normal_y_index = getFieldIndex(mesh.cloud, "normal_y");
		int normal_z_index = getFieldIndex(mesh.cloud, "normal_z");
		int curvature_index = getFieldIndex(mesh.cloud, "curvature");
		int rgba_index = getFieldIndex(mesh.cloud, "rgba");
		int rgb_index = getFieldIndex(mesh.cloud, "rgb");
		int diffuseAlbedo_index = getFieldIndex(mesh.cloud, "diffuseAlbedo");
		int specularAlbedo_index = getFieldIndex(mesh.cloud, "specularAlbedo");
		int specularSharpness_index = getFieldIndex(mesh.cloud, "specularSharpness");
		//int label_index = getFieldIndex(mesh.cloud, "label");
		
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;
		std::vector<float> normal_x;
		std::vector<float> normal_y;
		std::vector<float> normal_z;
		std::vector<float> curvature;
		std::vector<uint32_t> rgba;
		std::vector<float> diffuseAlbedo;
		std::vector<float> specularAlbedo;
		std::vector<float> specularSharpness;
		//std::vector<uint32_t> label;

		// number of points
		std::size_t numPoints = mesh.cloud.width * mesh.cloud.height;
		std::size_t pointSize = mesh.cloud.data.size() / numPoints;

		// number of faces
		std::size_t numFaces = mesh.polygons.size();

		// number of material
		//std::size_t numMaterials = 0;

		//
		// Write header
		{
			// Open file
			std::ofstream file;

			file.open(fileName.c_str(), std::ofstream::out);
			if (!file)
			{
				PRINT_ERROR("Can not open: " + fileName);
				return -1;
			}

			file << "ply";
			if (binary)
				file << "\nformat " << (mesh.cloud.is_bigendian ? "binary_big_endian" : "binary_little_endian") << " 1.0";
			else
				file << "\nformat ascii 1.0";
			file << "\ncomment PCL generated";

			// Vertices
			file << "\nelement vertex " << numPoints;

			//
			if (x_index != -1)
			{
				file << "\nproperty float x";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[x_index], pcl::PCLPointField::FLOAT32, &x))
					return -2;
			}

			if (y_index != -1)
			{
				file << "\nproperty float y";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[y_index], pcl::PCLPointField::FLOAT32, &y))
					return -2;
			}

			if (z_index != -1)
			{
				file << "\nproperty float z";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[z_index], pcl::PCLPointField::FLOAT32, &z))
					return -2;
			}

			if (normal_x_index != -1)
			{
				file << "\nproperty float nx";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[normal_x_index], pcl::PCLPointField::FLOAT32, &normal_x))
					return -2;
			}

			if (normal_y_index != -1)
			{
				file << "\nproperty float ny";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[normal_y_index], pcl::PCLPointField::FLOAT32, &normal_y))
					return -2;
			}

			if (normal_z_index != -1)
			{
				file << "\nproperty float nz";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[normal_z_index], pcl::PCLPointField::FLOAT32, &normal_z))
					return -2;
			}

			if (curvature_index != -1)
			{
				file << "\nproperty float curvature";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[curvature_index], pcl::PCLPointField::FLOAT32, &curvature))
					return -2;
			}

			if (rgba_index != -1)
			{
				file << "\nproperty uchar red"
					"\nproperty uchar green"
					"\nproperty uchar blue"
					"\nproperty uchar alpha";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[rgba_index], pcl::PCLPointField::UINT32, &rgba))
					return -2;
			}

			else if (rgb_index != -1)
			{
				file << "\nproperty uchar red"
					"\nproperty uchar green"
					"\nproperty uchar blue";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[rgb_index], pcl::PCLPointField::UINT32, &rgba))
					return -2;
			}

			if (diffuseAlbedo_index != -1)
			{
				file << "\nproperty float diffuseAlbedo";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[diffuseAlbedo_index], pcl::PCLPointField::FLOAT32, &diffuseAlbedo))
					return -2;
			}

			if (specularAlbedo_index != -1)
			{
				file << "\nproperty float specularAlbedo";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[specularAlbedo_index], pcl::PCLPointField::FLOAT32, &specularAlbedo))
					return -2;
			}

			if (specularSharpness_index != -1)
			{
				file << "\nproperty float specularSharpness";
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[specularSharpness_index], pcl::PCLPointField::FLOAT32, &specularSharpness))
					return -2;
			}

			/*if (label_index != -1)
			{
				file << "\nproperty int material_index";
				label.resize(numPoints);
				if (!Copy(numPoints, pointSize, mesh.cloud.data,
					mesh.cloud.fields[label_index], pcl::PCLPointField::UINT32, &label))
					return -2;

				for (std::vector<uint32_t>::iterator it = label.begin(); it != label.end(); ++it)
				{
					LabelUnion labelUnion;
					labelUnion.uint32 = *it;
					if (labelUnion.int32 != -1)
						*it += 1;
					else
						*it = 0;

					if (*it > numMaterials)
						numMaterials = *it;
				}
				numMaterials += 1;
			}*/

			// Faces
			file << "\nelement face " << numFaces;
			file << "\nproperty list uchar int vertex_indices";

			// Material
			/*if (numMaterials > 0)
			{
				file << "\nelement material " << numMaterials;
				file << "\nproperty float specular_coeff";
				file << "\nproperty float specular_power";
			}*/

			file << "\nend_header\n";
			file.close();
		}

		{
			std::ofstream file;

			if (binary)
			{
				file.open(fileName.c_str(), std::ios::out | std::ios::app | std::ios::binary);

				if (!file)
				{
					PRINT_ERROR("Can not open: " + fileName);
					return -1;
				}

				// Write down vertices
				for (size_t i = 0; i < numPoints; ++i)
				{
					if (x_index != -1) file.write(reinterpret_cast<const char*> (&x[i]), sizeof(float));
					if (y_index != -1) file.write(reinterpret_cast<const char*> (&y[i]), sizeof(float));
					if (z_index != -1) file.write(reinterpret_cast<const char*> (&z[i]), sizeof(float));

					if (normal_x_index != -1) file.write(reinterpret_cast<const char*> (&normal_x[i]), sizeof(float));
					if (normal_y_index != -1) file.write(reinterpret_cast<const char*> (&normal_y[i]), sizeof(float));
					if (normal_z_index != -1) file.write(reinterpret_cast<const char*> (&normal_z[i]), sizeof(float));
					if (curvature_index != -1) file.write(reinterpret_cast<const char*> (&curvature[i]), sizeof(float));

					if (rgba_index != -1)
					{
						ColorUnion color;
						color.uint32 = rgba[i];
						file.write(reinterpret_cast<const char*> (&color.rgba.r), sizeof(uint8_t));
						file.write(reinterpret_cast<const char*> (&color.rgba.g), sizeof(uint8_t));
						file.write(reinterpret_cast<const char*> (&color.rgba.b), sizeof(uint8_t));
						file.write(reinterpret_cast<const char*> (&color.rgba.a), sizeof(uint8_t));
					}

					else if (rgb_index != -1)
					{
						ColorUnion color;
						color.uint32 = rgba[i];
						file.write(reinterpret_cast<const char*> (&color.rgba.r), sizeof(uint8_t));
						file.write(reinterpret_cast<const char*> (&color.rgba.g), sizeof(uint8_t));
						file.write(reinterpret_cast<const char*> (&color.rgba.b), sizeof(uint8_t));
					}

					if (diffuseAlbedo_index != -1) file.write(reinterpret_cast<const char*> (&diffuseAlbedo[i]), sizeof(float));

					if (specularAlbedo_index != -1) file.write(reinterpret_cast<const char*> (&specularAlbedo[i]), sizeof(float));

					if (specularSharpness_index != -1) file.write(reinterpret_cast<const char*> (&specularSharpness[i]), sizeof(float));

					//if (label_index != -1) file.write(reinterpret_cast<const char*> (&label[i]), sizeof(float));

					file << '\n';
				}

				// Write down faces
				for (size_t i = 0; i < numFaces; i++)
				{
					unsigned char value = static_cast<unsigned char> (mesh.polygons[i].vertices.size());
					file.write(reinterpret_cast<const char*> (&value), sizeof(unsigned char));
					for (const int value : mesh.polygons[i].vertices)
						file.write(reinterpret_cast<const char*> (&value), sizeof(int));
				}

				// Write down materials
				/*for (size_t i = 0; i < numMaterials; i++)
				{
					float zero = 0;
					file.write(reinterpret_cast<const char*> (&zero), sizeof(float));
					file.write(reinterpret_cast<const char*> (&zero), sizeof(float));
				}*/
			}
			else
			{
				file.precision(precision);
				file.open(fileName.c_str(), std::ios::out | std::ios::app);

				if (!file)
				{
					PRINT_ERROR("Can not open: " + fileName);
					return -1;
				}

				// Write down vertices
				for (size_t i = 0; i < numPoints; ++i)
				{
					if (x_index != -1) file << x[i] << " ";
					if (y_index != -1) file << y[i] << " ";
					if (z_index != -1) file << z[i] << " ";

					if (normal_x_index != -1) file << normal_x[i] << " ";
					if (normal_y_index != -1) file << normal_y[i] << " ";
					if (normal_z_index != -1) file << normal_z[i] << " ";
					if (curvature_index != -1) file << curvature[i] << " ";

					if (rgba_index != -1)
					{
						ColorUnion color;
						color.uint32 = rgba[i];
						file << int(color.rgba.r) << " " << int(color.rgba.g) << " " << int(color.rgba.b) << " " << int(color.rgba.a) << " ";
					}

					else if (rgb_index != -1)
					{
						ColorUnion color;
						color.uint32 = rgba[i];
						file << int(color.rgba.r) << " " << int(color.rgba.g) << " " << int(color.rgba.b) << " ";
					}

					if (diffuseAlbedo_index != -1) file << diffuseAlbedo[i] << " ";

					if (specularAlbedo_index != -1) file << specularAlbedo[i] << " ";
					
					if (specularSharpness_index != -1) file << specularSharpness[i] << " ";

					//if (label_index != -1) file << label[i] << " ";

					file << '\n';
				}

				// Write down faces
				for (size_t i = 0; i < numFaces; i++)
				{
					file << mesh.polygons[i].vertices.size() << " ";
					for (size_t j = 0; j < mesh.polygons[i].vertices.size() - 1; ++j)
						file << mesh.polygons[i].vertices[j] << " ";
					file << mesh.polygons[i].vertices.back() << '\n';
				}

				// Write down materials
				/*for (size_t i = 0; i < numMaterials; i++)
				{
					file << 0 << " " << 0 << '\n';
				}*/
			}

			file.close();
		}

		return 0;
	}	

	void SyncMeshNormal(Mesh& mesh)
	{
		Pc<pcl::PointNormal> vertices;
		pcl::fromPCLPointCloud2(mesh.cloud, vertices);

		for (std::vector<pcl::Vertices>::iterator polyIT = mesh.polygons.begin(); polyIT != mesh.polygons.end(); ++polyIT)
		{
			switch (polyIT->vertices.size())
			{
			case 3:
			{
				uint32_t& a = polyIT->vertices[0];
				uint32_t& b = polyIT->vertices[1];
				uint32_t& c = polyIT->vertices[2];

				const pcl::PointNormal& va = vertices[a];
				const pcl::PointNormal& vb = vertices[b];
				const pcl::PointNormal& vc = vertices[c];

				Eigen::Vector3f tn = (va.getVector3fMap() - vb.getVector3fMap()).cross(va.getVector3fMap() - vc.getVector3fMap());

				if ((tn.dot(va.getNormalVector3fMap()) + tn.dot(vb.getNormalVector3fMap()) + tn.dot(vc.getNormalVector3fMap())) < 0)
				{
					uint32_t temp = b;
					b = c;
					c = temp;
				}

				break;
			}
			default:
			{
				PRINT_WARNING("polygon vertices size is not support");
				break;
			}
			}
		}
	}
};

