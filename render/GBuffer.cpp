#include "GBuffer.h"

cGBuffer::cGBuffer(void)
{
}

bool cGBuffer::Init(unsigned int width, unsigned int height)
{
	bool succ = true;
	mWidth = width;
	mHeight = height;

	// normal
	glGenTextures(1, &mTextures[eGBufferTexNormal]);
	glBindTexture(GL_TEXTURE_2D, mTextures[eGBufferTexNormal]);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, 0);
	glBindTexture(GL_TEXTURE_2D, 0);

	// depth buffer
	glGenTextures(1, &mTextures[eGBufferTexDepth]);
	glBindTexture(GL_TEXTURE_2D, mTextures[eGBufferTexDepth]);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

	float border_color[4] = { 1.f, 1.f, 1.f, 1.f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 
				0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

	// linked depth stencil and render texture
	glGenFramebuffers(1, &mObject);
	glBindFramebuffer(GL_FRAMEBUFFER, mObject);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
							GL_TEXTURE_2D, mTextures[eGBufferTexNormal], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 
							GL_TEXTURE_2D, mTextures[eGBufferTexDepth], 0);

	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, mTextures[ eGBufferTexDepth]);

	GLenum DrawBuffers[] = {GL_COLOR_ATTACHMENT0}; 
    glDrawBuffers(eGBufferTexMax - 1, DrawBuffers); // -1 for tex

	GLenum status;
	GLenum z = GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER;
	if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
		std::printf("texture is incomplete/n");
		succ = false;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	mTexture = mTextures[eGBufferTexNormal];
	mDepthStencil = mTextures[eGBufferTexDepth];

    return succ;
}

void cGBuffer::BindTex(GLint tex_slot, eGBufferTex tex_type)
{
	glActiveTexture(tex_slot);
	glBindTexture(GL_TEXTURE_2D, mTextures[tex_type]);
}

void cGBuffer::Reshape(int w, int h)
{
	mWidth = w;
	mHeight = h;
	if (mObject != 0) // 0 indicates the device's frame buffer, so no need to resize it
	{
		for (int i = 0; i < eGBufferTexMax; ++i)
		{
			glDeleteTextures(1, &mTextures[i]);
		}
		glDeleteFramebuffersEXT(1, &mObject);
		Init(w, h);
	}
}

cGBuffer::~cGBuffer(void)
{
}
