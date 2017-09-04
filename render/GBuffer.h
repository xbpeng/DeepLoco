#pragma once

#include "TextureDesc.h"

class cGBuffer : public cTextureDesc
{
public:
	enum eGBufferTex {
		eGBufferTexNormal,
		eGBufferTexDepth,
		eGBufferTexMax
    };

	cGBuffer(void);
	~cGBuffer(void);

	bool Init(unsigned int WindowWidth, unsigned int WindowHeight);
    void BindTex(GLint tex_slot, eGBufferTex tex_type);

	void Reshape(int w, int h);

private:
    GLuint mTextures[eGBufferTexMax];
};

