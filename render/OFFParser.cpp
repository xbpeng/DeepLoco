#include "render/OFFParser.h"

#include <sstream>

#include "render/MeshUtil.h"

bool cOFFParser::LoadMesh(const std::string& filename, cDrawMesh& out_mesh)
{
	const int vert_dim = 3;
	const int face_verts = 3;
	bool succ = true;

	std::fstream f_stream(filename);
	if (f_stream.good())
	{
		std::string str = "";
		AdvanceLine(f_stream, str);
		assert(str == "OFF");

		AdvanceLine(f_stream, str);
		std::stringstream str_stream(str);

		int num_verts = 0;
		int num_faces = 0;
		int num_edges = 0;
		str_stream >> num_verts;
		str_stream >> num_faces;
		str_stream >> num_edges;

		std::vector<float> vert_data;
		std::vector<int> face_data;
		vert_data.reserve(vert_dim * num_verts);
		face_data.reserve(face_verts * num_faces);

		for (int v = 0; v < num_verts; ++v)
		{
			bool valid_vert = AdvanceLine(f_stream, str);
			assert(valid_vert);
			str_stream = std::stringstream(str);

			for (int i = 0; i < vert_dim; ++i)
			{
				float val = 0;
				str_stream >> val;
				vert_data.push_back(val);
			}
		}
		assert(vert_data.size() == vert_dim * num_verts);

		for (int f = 0; f < num_faces; ++f)
		{
			bool valid_face = AdvanceLine(f_stream, str);
			assert(valid_face);
			str_stream = std::stringstream(str);

			int curr_vert_count = 0;
			str_stream >> curr_vert_count;
			assert(curr_vert_count == face_verts);

			for (int i = 0; i < face_verts; ++i)
			{
				int val = 0;
				str_stream >> val;
				face_data.push_back(val);
			}
		}
		assert(face_data.size() == face_verts * num_faces);

		cMeshUtil::BuildDrawMesh(vert_data.data(), static_cast<int>(vert_data.size()),
			face_data.data(), static_cast<int>(face_data.size()),
			&out_mesh);
	}
	else
	{
		succ = false;
	}

	return succ;
}

bool cOFFParser::AdvanceLine(std::fstream& f_stream, std::string& out_str)
{
	bool succ = false;
	std::string str = "";
	while (std::getline(f_stream, str))
	{
		if (str != "")
		{
			if (str[0] != '#')
			{
				out_str = str;
				succ = true;
				break;
			}
		}
	}
	return succ;
}