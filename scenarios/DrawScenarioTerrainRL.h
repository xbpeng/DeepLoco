#pragma once
#include <memory>
#include <GL/glew.h>

#include "scenarios/DrawScenario.h"

class cShader;
class cSkyBox;
class cPostProcessor;
class cTextureDesc;
class cShadowMap;
class cGBuffer;

class cDrawScenarioTerrainRL : public cDrawScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eDrawMode
	{
		eDrawMode2D,
		eDrawMode3D,
		eDrawModeMax
	};

	virtual ~cDrawScenarioTerrainRL();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);
	virtual void Reshape(int w, int h);

	virtual void SetOutputTex(const std::shared_ptr<cTextureDesc>& tex);

	virtual std::string BuildTextInfoStr() const;
	virtual void Shutdown();

protected:

	std::shared_ptr<cArgParser> mArgParser;

	tVector mCamDelta;
	bool mClearFilmStrip;

	// 3D shaders
	std::shared_ptr<cTextureDesc> mOutputTex;
	std::unique_ptr<cTextureDesc> mIntBufferTex;
	std::unique_ptr<cShadowMap> mShadowMap;
	std::unique_ptr<cShader> mLambertShaderMesh;
	std::unique_ptr<cShader> mShaderDepth;

	GLuint mTexResHandle;

	GLuint mLambertLightDirHandle;
	GLuint mLambertLightColourHandle;
	GLuint mLambertAmbientColourHandle;
	GLuint mLambertShadowProjHandle;
	GLuint mLambertMaterialDataHandle;
	GLuint mLambertFogColorHandle;
	GLuint mLambertFogDataHandle;

	GLuint mHBAOFocalLenLinMADHandle;
	GLuint mHBAOUVToViewABHandle;
	GLuint mHBAOTexResHandle;

	GLuint mSSRFocalLenLinMADHandle;
	GLuint mSSRUVToViewABHandle;
	GLuint mSSRTexResHandle;

	cCamera mShadowCam;

	std::unique_ptr<cTextureDesc> mGridTex0;
	std::unique_ptr<cTextureDesc> mGridTex1;

	cDrawScenarioTerrainRL(cCamera& cam);

	virtual void UpdateScene(double time_elapsed) = 0;

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;

	virtual void ResetCallback();

	virtual eDrawMode GetDrawMode() const;

	virtual void DrawScene();
	virtual void DrawGrid() const;
	virtual void DrawGroundMainScene();
	virtual void DrawCharacterMainScene();
	virtual void DrawObjsMainScene();
	virtual void DrawMiscMainScene();
	virtual void DrawGround() const;
	virtual void DrawCharacters() const;
	virtual void DrawObjs() const;
	virtual void DrawMisc() const;
	virtual void DrawInfo() const;

	virtual void EnableDraw3D(bool enable);
	virtual tVector GetVisOffset() const;
	virtual tVector GetLineColor() const;
	virtual tVector GetGroundColor() const;

	virtual void InitRenderResources();
	virtual bool LoadTextures();
	virtual void SetupMeshShader3D();
	virtual void DoShadowPass();
};