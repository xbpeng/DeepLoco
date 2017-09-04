#pragma once
#include "util/MathUtil.h"
#include <caffe/net.hpp>
#include <caffe/caffe.hpp>
#include <mutex>

class cNNSolver;

class cNeuralNet
{
public:
	typedef float tNNData;

	struct tProblem
	{
		tProblem();

		Eigen::MatrixXd mX;
		Eigen::MatrixXd mY;
		Eigen::MatrixXd mW;
		int mPassesPerStep;

		bool HasData() const;
		bool HasWeights() const;
		void Clear();
	};

	enum eOffsetScaleType
	{
		eOffsetScaleTypeNone,
		eOffsetScaleTypeFixed,
		eOffsetScaleTypeMax
	};

	static void PrintParams(const caffe::Net<tNNData>& net);
	static int CalcNumParams(const caffe::Net<tNNData>& net);
	static void CopyModel(const caffe::Net<tNNData>& src, caffe::Net<tNNData>& dst);
	static void CopyParams(const std::vector<caffe::Blob<tNNData>*>& src_params, const std::vector<caffe::Blob<tNNData>*>& dst_params);
	static bool CompareModel(const caffe::Net<tNNData>& a, const caffe::Net<tNNData>& b);
	static bool CompareParams(const std::vector<caffe::Blob<tNNData>*>& a_params, const std::vector<caffe::Blob<tNNData>*>& b_params);

	cNeuralNet();
	virtual ~cNeuralNet();

	virtual void LoadNet(const std::string& net_file);
	virtual void LoadModel(const std::string& model_file);
	virtual void LoadSolver(const std::string& solver_file);
	virtual void LoadScale(const std::string& scale_file);

	virtual void Clear();
	virtual void Train(const tProblem& prob);
	virtual double ForwardBackward(const tProblem& prob);
	virtual void StepSolver(int iters);

	virtual void ResetNet();
	virtual void ResetSolver();
	virtual void ResetWeights();

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual const Eigen::VectorXd& GetInputOffset() const;
	virtual const Eigen::VectorXd& GetInputScale() const;
	virtual const Eigen::VectorXd& GetOutputOffset() const;
	virtual const Eigen::VectorXd& GetOutputScale() const;

	virtual int GetProblemXSize() const;
	virtual int GetProblemYSize() const;

	virtual void Eval(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const;
	virtual void EvalBatch(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const;
	// y_diff should be the error you want to back propogate after calling Eval() to pass the values through the network
	virtual void Backward(const Eigen::VectorXd& y_diff, Eigen::VectorXd& out_x_diff) const;
	
	virtual int GetInputSize() const;
	virtual int GetOutputSize() const;
	virtual int GetBatchSize() const;
	virtual int CalcNumParams() const;

	virtual void OutputModel(const std::string& out_file) const;
	virtual void PrintParams() const;

	virtual bool HasNet() const;
	virtual bool HasSolver() const;
	virtual bool HasLayer(const std::string layer_name) const;
	virtual bool HasValidModel() const;

	virtual void NormalizeInput(Eigen::MatrixXd& X) const;
	virtual void NormalizeInput(Eigen::VectorXd& x) const;
	virtual void NormalizeInputDiff(Eigen::VectorXd& x_diff) const;
	virtual void UnnormalizeInput(Eigen::VectorXd& x) const;
	virtual void UnnormalizeInputDiff(Eigen::VectorXd& x_diff) const;
	virtual void NormalizeOutput(Eigen::VectorXd& y) const;
	virtual void NormalizeOutputDiff(Eigen::VectorXd& y_diff) const;
	virtual void UnnormalizeOutput(Eigen::VectorXd& y) const;
	virtual void UnnormalizeOutputDiff(Eigen::VectorXd& y_diff) const;

	virtual void CopyModel(const cNeuralNet& other);
	virtual void LerpModel(const cNeuralNet& other, double lerp);
	virtual void BlendModel(const cNeuralNet& other, double this_weight, double other_weight);
	virtual void BuildNetParams(caffe::NetParameter& out_params) const;
	virtual bool CompareModel(const cNeuralNet& other) const;
	virtual bool CompareGrad(const cNeuralNet& other) const; // just for debugging

	virtual void ForwardInjectNoisePrefilled(double mean, double stdev, const std::string& layer_name, Eigen::VectorXd& out_y) const;
	virtual void GetLayerState(const std::string& layer_name, Eigen::VectorXd& out_state) const;
	virtual void SetLayerState(const Eigen::VectorXd& state, const std::string& layer_name) const;

	virtual const std::vector<caffe::Blob<tNNData>*>& GetParams() const;
	virtual void SyncSolverParams();
	virtual void SyncNetParams();

	virtual void SetSolverErrWeights(const Eigen::MatrixXd& weights);
	virtual void CopyGrad(const cNeuralNet& other);

	virtual boost::shared_ptr<caffe::Net<tNNData>> GetTrainNet() const;

protected:
	class cCaffeNetWrapper : public caffe::Net<tNNData>
	{
	public:
		cCaffeNetWrapper(const std::string& net_file, caffe::Phase phase);
		virtual ~cCaffeNetWrapper();

		virtual int GetLayerIdx(const std::string& layer_name) const;
	};

	static std::mutex gOutputLock;
	static const int gXBlobIdx;
	static const int gYBlobIdx;
	static const int gWBlobIdx;

	bool mValidModel;
	
	std::unique_ptr<cCaffeNetWrapper> mNet;
	std::shared_ptr<cNNSolver> mSolver;
	std::string mNetFile;
	std::string mSolverFile;
	
	Eigen::VectorXd mInputOffset;
	Eigen::VectorXd mInputScale;
	Eigen::VectorXd mOutputOffset;
	Eigen::VectorXd mOutputScale;

	static const tNNData* GetBlobData(const caffe::Blob<tNNData>* blob);
	static tNNData* GetBlobDataMutable(caffe::Blob<tNNData>* blob);
	static const tNNData* GetBlobDiff(const caffe::Blob<tNNData>* blob);
	static tNNData* GetBlobDiffMutable(caffe::Blob<tNNData>* blob);

	virtual bool ValidOffsetScale() const;
	virtual void InitOffsetScale();
	virtual void InitTrainNet();

	virtual void FetchOutput(const std::vector<caffe::Blob<tNNData>*>& results_arr, Eigen::VectorXd& out_y) const;
	virtual void FetchInput(Eigen::VectorXd& out_x) const;
	virtual void EvalBatchNet(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const;
	virtual void EvalBatchSolver(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const;
	virtual bool HasTrainInputWeights() const;

	virtual const std::vector<caffe::Blob<tNNData>*>* GetTrainInputs() const;
	virtual void LoadTrainData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y);

	virtual tNNData* GetInputDataX() const;
	virtual tNNData* GetTrainInputDataX() const;
	virtual tNNData* GetTrainInputDataY() const;
	virtual tNNData* GetTrainInputDataW() const;

	virtual bool WriteData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, const std::string& out_file);
	virtual std::string GetOffsetScaleFile(const std::string& model_file) const;

	virtual void WriteOffsetScale(const std::string& scale_file) const;
	virtual const std::string& GetInputLayerName() const;
	virtual const std::string& GetOutputLayerName() const;
};
