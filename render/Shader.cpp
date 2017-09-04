#include "Shader.h"
#include <iostream>

cShader::cShader(void)
{
	mProg = -1;
}

cShader::~cShader(void)
{
	glDeleteProgram(mProg);
}

bool cShader::BuildShader(const std::string& vs_filename, const std::string& ps_filename)
{
	GLuint vs = LoadShader(vs_filename, GL_VERTEX_SHADER);
	GLuint ps = LoadShader(ps_filename, GL_FRAGMENT_SHADER);
	mProg = CreateShader(vs, ps);

	return mProg != -1;
}

GLuint cShader::LoadShader(const std::string& filename, GLenum shader_type)
{
	GLuint shader_handle = glCreateShader(shader_type);

	//read shader file
	std::ifstream shader_file(filename.c_str());
	std::cout<< "Compiling shader: " << filename.c_str() << std::endl;

	std::string shader_code((std::istreambuf_iterator<char>(shader_file)),
							 (std::istreambuf_iterator<char>()));
	
	const char* shader_code_str = shader_code.c_str();
	glShaderSource(shader_handle, 1, &shader_code_str, NULL);
	glCompileShader(shader_handle);
	// Check the compile status

	GLint compiled;
	glGetShaderiv(shader_handle, GL_COMPILE_STATUS, &compiled);
	if (!compiled)
	{
		std::cout<< "shader compilation failed: " << filename.c_str() << std::endl;
		
		// get log string length
		GLint max_len = 0;
		glGetShaderiv(shader_handle, GL_INFO_LOG_LENGTH, &max_len);

		char* error_log = new char[max_len];
		glGetShaderInfoLog(shader_handle, max_len, &max_len, error_log);

		std::cout << error_log << std::endl;

		delete[] error_log;
		glDeleteShader(shader_handle);
		shader_handle = -1;
	}

	return shader_handle;
}

GLuint cShader::CreateShader(GLuint& vsh_handle, GLuint& psh_handle)
{
	GLuint shader_program;
	GLint valid_status;

	shader_program = glCreateProgram();
	glAttachShader(shader_program, vsh_handle);
	glAttachShader(shader_program, psh_handle);
	glLinkProgram(shader_program);
	glGetProgramiv(shader_program, GL_LINK_STATUS, &valid_status);

	// valid program
	glGetProgramiv(shader_program, GL_LINK_STATUS, &valid_status);
	if (!valid_status)
	{
		std::cout << "shader linking failed\n";
	}
	else
	{
		glValidateProgram(shader_program);
		glGetProgramiv(shader_program, GL_VALIDATE_STATUS, &valid_status);
		if (!valid_status)
		{
			std::cout << "shader validation failed\n";
		}
	}
	
	if (!valid_status)
	{
		shader_program = -1;
	}

	return shader_program;
}

void cShader::BindShaderAttribute(GLuint& attribute_handle, const std::string& attribute_name)
{
	attribute_handle = glGetAttribLocation(mProg, attribute_name.c_str());
	if (attribute_handle == -1)
	{
		std::cout << "failed to bind shader attribute " << attribute_name.c_str() << std::endl;
	}
}

void cShader::BindShaderUniform(GLuint& uniform_handle, const std::string& uniform_name)
{
	uniform_handle = glGetUniformLocation(mProg, uniform_name.c_str());
	if (uniform_handle == -1)
	{
		std::cout << "failed to bind shader uniform parameter " << uniform_name.c_str() << std::endl;
	}
}

void cShader::Bind() const
{
	glUseProgram(mProg);
}

void cShader::Unbind() const
{
	glUseProgram(0);
}

void cShader::SetUniform3(GLuint handle, const tVector& data) const
{
	float f_data[] = {static_cast<float>(data[0]), 
					  static_cast<float>(data[1]), 
					  static_cast<float>(data[2])};
	glProgramUniform3fv(mProg, handle, 1, f_data);
}

void cShader::SetUniform4(GLuint handle, const tVector& data) const
{
	float f_data[] = {static_cast<float>(data[0]), 
					  static_cast<float>(data[1]), 
					  static_cast<float>(data[2]),
					  static_cast<float>(data[3])};
	glProgramUniform4fv(mProg, handle, 1, f_data);
}

void cShader::SetUniform2(GLuint handle, const tVector& data) const
{
	float f_data[] = { static_cast<float>(data[0]),
						static_cast<float>(data[1]) };
	glProgramUniform2fv(mProg, handle, 1, f_data);
}

GLuint cShader::GetProg() const
{
	return mProg;
}