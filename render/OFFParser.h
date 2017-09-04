#include "render/DrawMesh.h"

class cOFFParser
{
public:
	static bool LoadMesh(const std::string& filename, cDrawMesh& out_mesh);

protected:
	static bool AdvanceLine(std::fstream& f_stream, std::string& out_str);
};