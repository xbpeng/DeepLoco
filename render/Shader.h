#pragma once

#include <stdlib.h>
#include <GL/glew.h>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <fstream>

#include "util/MathUtil.h"

class cShader
{
public:
	cShader(void);
	~cShader(void);

	virtual bool BuildShader(const std::string& vs_filename, const std::string& ps_filename);
	virtual void BindShaderAttribute(GLuint& attribute_handle, const std::string& attribute_name);
	virtual void BindShaderUniform(GLuint& uniform_handle, const std::string& uniform_name);

	virtual void Bind() const;
	virtual void Unbind() const;

	virtual void SetUniform3(GLuint handle, const tVector& data) const;
	virtual void SetUniform4(GLuint handle, const tVector& data) const;
	virtual void SetUniform2(GLuint handle, const tVector& data) const;

	virtual GLuint GetProg() const;

protected:
	GLuint mProg;

	virtual GLuint LoadShader(const std::string& filename, GLenum shader_type);
	virtual GLuint CreateShader(GLuint& vsh_handle, GLuint& psh_handle);
};
