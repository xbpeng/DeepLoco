#pragma once

#include "learning/NeuralNet.h"
#include "CharController.h"

class cNNController : public cCharController
{
public:
	virtual ~cNNController();

	virtual void Init(cSimCharacter* character);

	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual int GetNetInputSize() const;
	virtual int GetNetOutputSize() const;

	virtual void RecordPoliState(Eigen::VectorXd& out_state) const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	virtual double CalcActionLogp() const;
	virtual double CalcReward() const;

	virtual bool IsOffPolicy() const;

	virtual bool LoadNet(const std::string& net_file);
	virtual void LoadModel(const std::string& model_file);
	virtual void LoadScale(const std::string& scale_file);
	virtual void CopyNet(const cNeuralNet& net);
	virtual void SaveNet(const std::string& out_file) const;

	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void SetNNInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual void SetNNOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const;
	virtual const std::unique_ptr<cNeuralNet>& GetNet() const;

protected:
	std::unique_ptr<cNeuralNet> mNet;

	cNNController();
	virtual bool HasNet() const;
	virtual void LoadNetIntern(const std::string& net_file);
	virtual void BuildNet(std::unique_ptr<cNeuralNet>& out_net) const;
};