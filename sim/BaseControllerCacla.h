#pragma once

#include "sim/TerrainRLCharController.h"

class cBaseControllerCacla : public virtual cTerrainRLCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cBaseControllerCacla();
	virtual ~cBaseControllerCacla();
	
	virtual void Init(cSimCharacter* character);
	
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	
	virtual bool ValidCritic() const;
	virtual bool LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticModel(const std::string& model_file);

	virtual void CopyNet(const cNeuralNet& net);
	virtual void CopyActorNet(const cNeuralNet& net);
	virtual void CopyCriticNet(const cNeuralNet& net);

	virtual const std::unique_ptr<cNeuralNet>& GetActor() const;
	virtual const std::unique_ptr<cNeuralNet>& GetCritic() const;

	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;

	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetNNInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetNNOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;

	virtual void BuildActorInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;

	virtual void BuildCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;

protected:
	std::unique_ptr<cNeuralNet> mCriticNet;

	virtual void BuildCriticNet(std::unique_ptr<cNeuralNet>& out_net) const;
	
	virtual bool ShouldExplore() const;
	virtual void DecideAction(tAction& out_action);
	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action);
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);
	virtual void EvalNet(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const;

	virtual void RecordVal();
	virtual void BuildCriticInput(Eigen::VectorXd& out_x) const;

	virtual void FetchExpNoiseScale(Eigen::VectorXd& out_noise) const;
	virtual void ApplyExpNoise(tAction& out_action);
};