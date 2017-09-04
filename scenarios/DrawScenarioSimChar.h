#pragma once
#include "scenarios/DrawScenarioSimInteractive.h"
#include "scenarios/ScenarioSimChar.h"
#include "sim/CharTracer.h"

class cDrawMesh;

class cDrawScenarioSimChar : public cDrawScenarioSimInteractive
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioSimChar(cCamera& cam);
	virtual ~cDrawScenarioSimChar();

	virtual void Init();
	
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	virtual void Keyboard(unsigned char key, int x, int y);

	virtual void EnableCharDrawShapes(bool enable);

	virtual std::string BuildTextInfoStr() const;
	virtual void Shutdown();

	virtual std::string GetName() const;

protected:
	std::shared_ptr<cScenarioSimChar> mScene;
	
	bool mDrawInfo;
	bool mDrawPoliInfo;
	bool mDrawFeatures;
	bool mDrawPolicyPlots;
	bool mEnableCharDrawShapes;
	bool mPauseSim;

	bool mEnableTrace;
	cCharTracer mTracer;
	std::vector<int> mTraceHandles;

	std::unique_ptr<cDrawMesh> mGroundDrawMesh;
	size_t mPrevGroundUpdateCount;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void SetupScene(std::shared_ptr<cScenarioSimChar>& out_scene);
	virtual void UpdateScene(double time_elapsed);

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;

	virtual void UpdateTracer(double time_elapsed);
	virtual void UpdateGroundDrawMesh();
	virtual void ResetCallback();
	virtual void PauseSim(bool pause);

	virtual void InitTracer();
	virtual int AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
								const tVectorArr& cols);
	virtual void ToggleTrace();
	virtual eDrawMode GetDrawMode() const;

	virtual void DrawGround() const;
	virtual void DrawCharacters() const;
	virtual void DrawCharacter(const std::shared_ptr<cSimCharacter>& character) const;
	virtual void DrawTrace() const;
	virtual void DrawObjs() const;
	virtual void DrawMisc() const;
	virtual void DrawInfo() const;

	virtual void EnableDraw3D(bool enable);

	virtual void DrawCoM() const;
	virtual void DrawTorque() const;
	virtual void DrawHeading() const;
	virtual void DrawPerturbs() const;
	virtual void DrawCtrlInfo() const;
	virtual void DrawPoliInfo() const;
	virtual void DrawFeatures() const;
	virtual void DrawPolicyPlots() const;
	virtual void DrawGroundDynamicObstacles3D(const std::shared_ptr<cGround>& ground) const;

	virtual const std::shared_ptr<cScenarioSimChar>& GetScene() const;
	virtual std::string GetOutputCharFile() const;
	virtual void OutputCharState(const std::string& out_file) const;

	virtual void SpawnProjectile();
	virtual void SpawnBigProjectile();

	virtual void BuildGroundDrawMesh();
};