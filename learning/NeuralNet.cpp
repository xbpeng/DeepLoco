#include "NeuralNet.h"
#include <caffe/util/hdf5.hpp>
#include "util/json/json.h"

#include "util/Util.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "NNSolver.h"

const std::string gInputOffsetKey = "InputOffset";
const std::string gInputScaleKey = "InputScale";
const std::string gOutputOffsetKey = "OutputOffset";
const std::string gOutputScaleKey = "OutputScale";
const std::string gInputLayerName = "input";
const std::string gOutputLayerName = "output";

const int cNeuralNet::gXBlobIdx = 0;
const int cNeuralNet::gYBlobIdx = 1;
const int cNeuralNet::gWBlobIdx = 2;

std::mutex cNeuralNet::gOutputLock;

cNeuralNet::tProblem::tProblem()
{
	mX.resize(0, 0);
	mY.resize(0, 0);
	mPassesPerStep = 1;
}

bool cNeuralNet::tProblem::HasData() const
{
	return mX.size() > 0;
}

bool cNeuralNet::tProblem::HasWeights() const
{
	return mW.size() > 0 && (mW.rows() == mY.rows()) && (mW.cols() == mY.cols());
}

void cNeuralNet::tProblem::Clear()
{
	mX.resize(0, 0);
	mY.resize(0, 0);
	mW.resize(0, 0);
}

cNeuralNet::cNeuralNet()
{
	Clear();
	mNetFile = "";
	mSolverFile = "";
}

cNeuralNet::~cNeuralNet()
{
}

void cNeuralNet::LoadNet(const std::string& net_file)
{
	if (net_file != "")
	{
		mNetFile = net_file;
		mNet = std::unique_ptr<cCaffeNetWrapper>(new cCaffeNetWrapper(net_file, caffe::TEST));
		mValidModel = false;

		// hack
		int num_params = CalcNumParams();

		if (!ValidOffsetScale())
		{
			InitOffsetScale();
		}

		if (HasSolver())
		{
			SyncNetParams();
		}
	}
}

void cNeuralNet::LoadModel(const std::string& model_file)
{
	if (model_file != "")
	{
		if (HasNet())
		{
			mNet->CopyTrainedLayersFromHDF5(model_file);
			LoadScale(GetOffsetScaleFile(model_file));
			SyncSolverParams();

			mValidModel = true;
		}
		else if (HasSolver())
		{
			auto net = GetTrainNet();
			net->CopyTrainedLayersFromHDF5(model_file);
			LoadScale(GetOffsetScaleFile(model_file));
			SyncNetParams();

			mValidModel = true;
		}
		else
		{
			printf("Net structure has not been initialized\n");
			assert(false);
		}
	}
}

void cNeuralNet::LoadSolver(const std::string& solver_file)
{
	if (solver_file != "")
	{
		mSolverFile = solver_file;

		cNNSolver::BuildSolver(solver_file, mSolver);

		if (!ValidOffsetScale())
		{
			InitOffsetScale();
		}

		if (HasNet())
		{
			SyncSolverParams();
		}

		InitTrainNet();
	}
}

void cNeuralNet::LoadScale(const std::string& scale_file)
{
	std::ifstream f_stream(scale_file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	int input_size = GetInputSize();
	if (succ && !root[gInputOffsetKey].isNull())
	{
		Eigen::VectorXd offset;
		succ &= cJsonUtil::ReadVectorJson(root[gInputOffsetKey], offset);

		int offset_size = static_cast<int>(offset.size());
		if (offset_size == input_size)
		{
			mInputOffset = offset;
		}
		else
		{
			printf("Invalid input offset size, expecting %i, but got %i\n", input_size, offset_size);
			assert(false);
			succ = false;
		}
	}

	if (succ && !root[gInputScaleKey].isNull())
	{
		Eigen::VectorXd scale;
		succ &= cJsonUtil::ReadVectorJson(root[gInputScaleKey], scale);

		int scale_size = static_cast<int>(scale.size());
		if (scale_size == input_size)
		{
			mInputScale = scale;
		}
		else
		{
			printf("Invalid input scale size, expecting %i, but got %i\n", input_size, scale_size);
			succ = false;
		}
	}

	int output_size = GetOutputSize();
	if (succ && !root[gOutputOffsetKey].isNull())
	{
		Eigen::VectorXd offset;
		succ &= cJsonUtil::ReadVectorJson(root[gOutputOffsetKey], offset);

		int offset_size = static_cast<int>(offset.size());
		if (offset_size == output_size)
		{
			mOutputOffset = offset;
		}
		else
		{
			printf("Invalid output offset size, expecting %i, but got %i\n", output_size, offset_size);
			succ = false;
		}
	}

	if (succ && !root[gOutputScaleKey].isNull())
	{
		Eigen::VectorXd scale;
		succ &= cJsonUtil::ReadVectorJson(root[gOutputScaleKey], scale);

		int scale_size = static_cast<int>(scale.size());
		if (scale_size == output_size)
		{
			mOutputScale = scale;
		}
		else
		{
			printf("Invalid output scale size, expecting %i, but got %i\n", output_size, scale_size);
			succ = false;
		}
	}
}

void cNeuralNet::Clear()
{
	mNet.reset();
	mSolver.reset();
	mValidModel = false;

	mInputOffset.resize(0);
	mInputScale.resize(0);
	mOutputOffset.resize(0);
	mOutputScale.resize(0);
}

void cNeuralNet::Train(const tProblem& prob)
{
	if (HasSolver())
	{
		LoadTrainData(prob.mX, prob.mY);

		if (prob.HasWeights())
		{
			SetSolverErrWeights(prob.mW);
		}

		int batch_size = GetBatchSize();
		int num_batches = static_cast<int>(prob.mX.rows()) / batch_size;
		
		StepSolver(prob.mPassesPerStep * num_batches);
	}
	else
	{
		printf("Solver has not been initialized\n");
		assert(false);
	}
}

double cNeuralNet::ForwardBackward(const tProblem& prob)
{
	double loss = 0;
	if (HasSolver())
	{
		LoadTrainData(prob.mX, prob.mY);
		loss = mSolver->ForwardBackward();
	}
	else
	{
		printf("Solver has not been initialized\n");
		assert(false);
	}
	return loss;
}

void cNeuralNet::StepSolver(int iters)
{
	//TIMER_PRINT_BEG(STEP_SOLVER)
	mSolver->ApplySteps(iters);
	//TIMER_PRINT_END(STEP_SOLVER)

	if (HasNet())
	{
		SyncNetParams();
	}
	mValidModel = true;
}

void cNeuralNet::ResetNet()
{
	LoadNet(mNetFile);
}

void cNeuralNet::ResetSolver()
{
	LoadSolver(mSolverFile);
}

void cNeuralNet::ResetWeights()
{
	mNet.reset();
	mSolver.reset();
	ResetNet();
	ResetSolver();
}

void cNeuralNet::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(offset.size() == GetInputSize());
	assert(scale.size() == GetInputSize());
	mInputOffset = offset;
	mInputScale = scale;
}

void cNeuralNet::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(offset.size() == GetOutputSize());
	assert(scale.size() == GetOutputSize());
	mOutputOffset = offset;
	mOutputScale = scale;
}

const Eigen::VectorXd& cNeuralNet::GetInputOffset() const
{
	return mInputOffset;
}

const Eigen::VectorXd& cNeuralNet::GetInputScale() const
{
	return mInputScale;
}

const Eigen::VectorXd& cNeuralNet::GetOutputOffset() const
{
	return mOutputOffset;
}

const Eigen::VectorXd& cNeuralNet::GetOutputScale() const
{
	return mOutputScale;
}

int cNeuralNet::GetProblemXSize() const
{
	return GetInputSize();
}

int cNeuralNet::GetProblemYSize() const
{
	return GetOutputSize();
}

void cNeuralNet::Eval(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const
{
	const int input_size = GetInputSize();
	assert(HasNet());
	assert(x.size() == input_size);

	tNNData* input_data = GetInputDataX();
	bool valid_offset_scale = ValidOffsetScale();
	for (int i = 0; i < input_size; ++i)
	{
		double val = x[i];
		if (valid_offset_scale)
		{
			val = mInputScale[i] * (val + mInputOffset[i]);
		}
		input_data[i] = val;
	}

	const std::vector<caffe::Blob<tNNData>*>& result_arr = mNet->Forward();
	FetchOutput(result_arr, out_y);
}

void cNeuralNet::EvalBatch(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const
{
	// hack hack hack
	if (HasSolver() && GetBatchSize() > 1)
	{
		EvalBatchSolver(X, out_Y);
	}
	else
	{
		EvalBatchNet(X, out_Y);
	}
}

void cNeuralNet::Backward(const Eigen::VectorXd& y_diff, Eigen::VectorXd& out_x_diff) const
{
	assert(HasNet());
	int output_size = GetOutputSize();
	const auto& top_vec = mNet->top_vecs();
	const auto& top_blob = top_vec[top_vec.size() - 1][0];
	assert(y_diff.size() == output_size);
	assert(y_diff.size() == top_blob->count());
	auto top_data = GetBlobDiffMutable(top_blob);

	// normalization is weird but the math seems to work out this way
	Eigen::VectorXd norm_y_diff = y_diff;
	UnnormalizeOutputDiff(norm_y_diff);

	for (int i = 0; i < output_size; ++i)
	{
		top_data[i] = norm_y_diff[i];
	}

	mNet->ClearParamDiffs();
	mNet->Backward();

	auto bottom_vecs = mNet->bottom_vecs();
	// The first layer is the train version and the second is the test version where gradients are computed
	const auto& bottom_blob = bottom_vecs[1][gXBlobIdx]; 
	const tNNData* bottom_data = GetBlobDiff(bottom_blob);

	int input_size = GetInputSize();
	assert(bottom_blob->count() == input_size);
	out_x_diff.resize(input_size);

	for (int i = 0; i < input_size; ++i)
	{
		out_x_diff[i] = bottom_data[i];
	}
	
	NormalizeInputDiff(out_x_diff);
}

void cNeuralNet::EvalBatchNet(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const
{
	assert(HasNet());
	int num_data = static_cast<int>(X.rows());
	Eigen::VectorXd x;
	Eigen::VectorXd y;

	out_Y.resize(num_data, GetOutputSize());
	for (int i = 0; i < num_data; ++i)
	{
		x = X.row(i);
		Eval(x, y);
		out_Y.row(i) = y;
	}
}

void cNeuralNet::EvalBatchSolver(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const
{
	assert(HasSolver());
	boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> net = GetTrainNet();

	const int num_inputs = static_cast<int>(X.rows());
	const int input_size = GetInputSize();
	const int output_size = GetOutputSize();
	assert(X.cols() == input_size);

	int batch_size = GetBatchSize();
	int num_data = static_cast<int>(X.rows());
	int num_batches = static_cast<int>(std::ceil((1.0 * X.rows()) / batch_size));
	out_Y.resize(num_data, output_size);

	tNNData* input_data = GetTrainInputDataX();
	for (int b = 0; b < num_batches; ++b)
	{
		for (int i = 0; i < batch_size; ++i)
		{
			int data_idx = b * batch_size + i;
			if (data_idx >= num_data)
			{
				break;
			}

			auto curr_data = X.row(data_idx);
			for (int j = 0; j < input_size; ++j)
			{
				double val = curr_data[j];
				if (ValidOffsetScale())
				{
					val += mInputOffset[j];
					val = val * mInputScale[j];
				}
				input_data[i * input_size + j] = val;
			}
		}

		net->Forward();

		const std::string& output_layer_name = GetOutputLayerName();
		const auto output_blob = net->blob_by_name(output_layer_name);
		int output_blob_count = output_blob->count();
		assert(output_blob_count == batch_size * output_size);

		auto output_blob_data = GetBlobData(output_blob.get());
		for (int i = 0; i < batch_size; ++i)
		{
			int data_idx = b * batch_size + i;
			auto curr_data = out_Y.row(data_idx);

			for (int j = 0; j < output_size; ++j)
			{
				double val = output_blob_data[i * output_size + j];
				if (ValidOffsetScale())
				{
					val /= mOutputScale[j];
					val -= mOutputOffset[j];
				}
				curr_data(j) = val;
			}
		}
	}
}

bool cNeuralNet::HasTrainInputWeights() const
{
	const auto* input_blobs = GetTrainInputs();
	bool has_weights = false;
	if (input_blobs != nullptr)
	{
		has_weights = input_blobs->size() > gWBlobIdx;
	}
	return has_weights;
}


int cNeuralNet::GetInputSize() const
{
	int size = 0;
	if (HasNet())
	{
		size = mNet->input_blobs()[gXBlobIdx]->count();
	}
	else if (HasSolver())
	{
		auto net = GetTrainNet();
		auto input_blob = net->blob_by_name(GetInputLayerName());
		size = input_blob->count();
		size /= GetBatchSize();
	}
	return size;
}

int cNeuralNet::GetOutputSize() const
{
	int size = 0;
	if (HasNet())
	{
		auto out_blobs = mNet->output_blobs();
		size = out_blobs[out_blobs.size() - 1]->count();
	}
	else if (HasSolver())
	{
		auto net = GetTrainNet();
		auto output_blob = net->blob_by_name(GetOutputLayerName());
		size = output_blob->count();
		size /= GetBatchSize();
	}

	return size;
}

int cNeuralNet::GetBatchSize() const
{
	int batch_size = 0;
	if (HasSolver())
	{
		const auto& input_blobs = GetTrainInputs();
		const auto& input_blob = (*input_blobs)[gXBlobIdx];
		batch_size = input_blob->shape(0);
	}
	return batch_size;
}

int cNeuralNet::CalcNumParams() const
{
	int num_params = 0;
	if (HasNet())
	{
		num_params = CalcNumParams(*mNet);
	}
	return num_params;
}

void cNeuralNet::OutputModel(const std::string& out_file) const
{
	if (HasNet())
	{
		{
			// arg, hdf5 doesn't seem to be threadsafe
			std::lock_guard<std::mutex> output_lock(gOutputLock);
			mNet->ToHDF5(out_file);
		}
		std::string scale_file = GetOffsetScaleFile(out_file);
		WriteOffsetScale(scale_file);
	}
	else
	{
		printf("No valid net to output\n");
	}
}

void cNeuralNet::PrintParams() const
{
	PrintParams(*mNet);
}

void cNeuralNet::PrintParams(const caffe::Net<tNNData>& net)
{
	const auto& param_blobs = net.learnable_params();
	int num_blobs = static_cast<int>(param_blobs.size());

	for (int b = 0; b < num_blobs; ++b)
	{
		printf("Params %i:\n", b);
		auto blob = param_blobs[b];
		auto blob_data = GetBlobData(blob);
		int blob_count = blob->count();
		for (int i = 0; i < blob_count; ++i)
		{
			printf("%.5f\n", blob_data[i]);
		}
	}
}

int cNeuralNet::CalcNumParams(const caffe::Net<tNNData>& net)
{
	auto layers = net.layers();
	const auto& param_blobs = net.learnable_params();
	int num_params = 0;
	int num_blobs = static_cast<int>(param_blobs.size());

	for (int b = 0; b < num_blobs; ++b)
	{
		const auto& blob = param_blobs[b];
		int count = blob->count();
		num_params += count;
	}

	return num_params;
}

void cNeuralNet::CopyModel(const caffe::Net<tNNData>& src, caffe::Net<tNNData>& dst)
{
	const auto& src_params = src.learnable_params();
	const auto& dst_params = dst.learnable_params();
	CopyParams(src_params, dst_params);
}

void cNeuralNet::CopyParams(const std::vector<caffe::Blob<tNNData>*>& src_params, const std::vector<caffe::Blob<tNNData>*>& dst_params)
{
	int num_blobs = static_cast<int>(src_params.size());
	for (int b = 0; b < num_blobs; ++b)
	{
		auto src_blob = src_params[b];
		auto dst_blob = dst_params[b];
		int src_blob_count = src_blob->count();
		int dst_blob_count = dst_blob->count();
		assert(src_blob_count == dst_blob_count);
		dst_blob->CopyFrom(*src_blob);
	}
}

bool cNeuralNet::CompareModel(const caffe::Net<tNNData>& a, const caffe::Net<tNNData>& b)
{
	const auto& a_params = a.learnable_params();
	const auto& b_params = b.learnable_params();
	return CompareParams(a_params, b_params);
}

bool cNeuralNet::CompareParams(const std::vector<caffe::Blob<tNNData>*>& a_params, const std::vector<caffe::Blob<tNNData>*>& b_params)
{
	int num_blobs = static_cast<int>(a_params.size());
	for (int i = 0; i < num_blobs; ++i)
	{
		auto a_blob = a_params[i];
		auto b_blob = b_params[i];

		auto a_blob_data = GetBlobData(a_blob);
		auto b_blob_data = GetBlobData(b_blob);
		int a_blob_count = a_blob->count();
		int b_blob_count = b_blob->count();

		if (a_blob_count == b_blob_count)
		{
			for (int j = 0; j < a_blob_count; ++j)
			{
				double a_val = a_blob_data[j];
				double b_val = b_blob_data[j];
				if (a_val != b_val)
				{
					return false;
				}
			}
		}
		else
		{
			assert(false); // param size mismatch
		}
	}
	return true;
}

bool cNeuralNet::HasNet() const
{
	return mNet != nullptr;
}

bool cNeuralNet::HasSolver() const
{
	return mSolver != nullptr;
}

bool cNeuralNet::HasLayer(const std::string layer_name) const
{
	if (HasNet())
	{
		return mNet->has_blob(layer_name) && mNet->has_layer(layer_name);
	}
	return false;
}

bool cNeuralNet::HasValidModel() const
{
	return mValidModel;
}

void cNeuralNet::CopyModel(const cNeuralNet& other)
{
	CopyParams(other.GetParams(), GetParams());

	mInputOffset = other.GetInputOffset();
	mInputScale = other.GetInputScale();
	mOutputOffset = other.GetOutputOffset();
	mOutputScale = other.GetOutputScale();

	SyncSolverParams();
	mValidModel = true;
}

void cNeuralNet::LerpModel(const cNeuralNet& other, double lerp)
{
	BlendModel(other, 1 - lerp, lerp);
}

void cNeuralNet::BlendModel(const cNeuralNet& other, double this_weight, double other_weight)
{
	const auto& src_params = other.GetParams();
	const auto& dst_params = GetParams();

	int num_blobs = static_cast<int>(src_params.size());
	for (int b = 0; b < num_blobs; ++b)
	{
		auto src_blob = src_params[b];
		auto dst_blob = dst_params[b];

		const tNNData* src_blob_data = GetBlobData(src_blob);
		tNNData* dst_blob_data = GetBlobDataMutable(dst_blob);
		int src_blob_count = src_blob->count();
		int dst_blob_count = dst_blob->count();
		assert(src_blob_count == dst_blob_count);

		for (int i = 0; i < src_blob_count; ++i)
		{
			dst_blob_data[i] = this_weight * dst_blob_data[i] + other_weight * src_blob_data[i];
		}
	}

	SyncSolverParams();
	mValidModel = true;
}

void cNeuralNet::BuildNetParams(caffe::NetParameter& out_params) const
{
	mNet->ToProto(&out_params);
}

bool cNeuralNet::CompareModel(const cNeuralNet& other) const
{
	bool same = CompareParams(other.GetParams(), GetParams());

	same &= mInputOffset.isApprox(other.GetInputOffset(), 0);
	same &= mInputScale.isApprox(other.GetInputScale(), 0);
	same &= mOutputOffset.isApprox(other.GetOutputOffset(), 0);
	same &= mOutputScale.isApprox(other.GetOutputScale(), 0);

	return same;
}

bool cNeuralNet::CompareGrad(const cNeuralNet& other) const
{
	assert(HasSolver());
	assert(other.HasSolver());
	auto other_net = other.GetTrainNet();
	auto this_net = GetTrainNet();

	auto other_params = other_net->learnable_params();
	auto this_params = this_net->learnable_params();
	assert(other_params.size() == this_params.size());

	bool same = true;
	double max_diff = 0;
	for (size_t i = 0; i < this_params.size(); ++i)
	{
		auto other_blob = other_params[i];
		auto this_blob = this_params[i];
		assert(other_blob->count() == this_blob->count());

		auto other_diff = GetBlobDiff(other_blob);
		auto this_diff = GetBlobDiffMutable(this_blob);
		for (int j = 0; j < this_blob->count(); ++j)
		{
			double a_val = this_diff[j];
			double b_val = other_diff[j];
			max_diff = std::max(max_diff, std::abs(a_val - b_val));
			if (a_val != b_val)
			{
				same = false;
				return false;
			}
		}
	}
	return same;
}

void cNeuralNet::ForwardInjectNoisePrefilled(double mean, double stdev, const std::string& layer_name, Eigen::VectorXd& out_y) const
{
	// assume the Eval has already been called which fills the blobs in the network using a given input
	if (HasLayer(layer_name))
	{
		auto blob = mNet->blob_by_name(layer_name);

		tNNData* data = GetBlobDataMutable(blob.get());
		const int data_size = blob->count();

		for (int i = 0; i < data_size; ++i)
		{
			double noise = cMathUtil::RandDoubleNorm(mean, stdev);
			data[i] += noise;
		}

		int layer_idx = mNet->GetLayerIdx(layer_name);
		++layer_idx;
		mNet->ForwardFrom(layer_idx);

		const std::vector<caffe::Blob<tNNData>*>& result_arr = mNet->output_blobs();
		FetchOutput(result_arr, out_y);
	}
	else
	{
		printf("Can't find layer named %s\n", layer_name.c_str());
		assert(false); // layer not found
	}
}

void cNeuralNet::GetLayerState(const std::string& layer_name, Eigen::VectorXd& out_state) const
{
	auto blob = mNet->blob_by_name(layer_name);
	if (blob != nullptr)
	{
		const tNNData* data = GetBlobData(blob.get());
		const int data_size = blob->count();

		out_state.resize(data_size);
		for (int i = 0; i < data_size; ++i)
		{
			out_state[i] = data[i];
		}
	}
	else
	{
		printf("Can't find layer named %s\n", layer_name.c_str());
		assert(false); // layer not found
	}
}

void cNeuralNet::SetLayerState(const Eigen::VectorXd& state, const std::string& layer_name) const
{
	auto blob = mNet->blob_by_name(layer_name);
	if (blob != nullptr)
	{
		tNNData* data = GetBlobDataMutable(blob.get());
		const int data_size = blob->count();
		assert(state.size() == data_size);

		for (int i = 0; i < data_size; ++i)
		{
			data[i] = state[i];
		}
	}
	else
	{
		printf("Can't find layer named %s\n", layer_name.c_str());
		assert(false); // layer not found
	}
}

const std::vector<caffe::Blob<cNeuralNet::tNNData>*>& cNeuralNet::GetParams() const
{
	if (HasNet())
	{
		return mNet->learnable_params();
	}
	else
	{
		auto net = GetTrainNet();
		return net->learnable_params();
	}
}

void cNeuralNet::SyncSolverParams()
{
	if (HasSolver() && HasNet())
	{
		CopyModel(*mNet, *GetTrainNet());
	}
}

void cNeuralNet::SyncNetParams()
{
	if (HasSolver() && HasNet())
	{
		CopyModel(*GetTrainNet(), *mNet);
	}
}

void cNeuralNet::SetSolverErrWeights(const Eigen::MatrixXd& weights)
{
	int num_data = static_cast<int>(weights.rows());
	int data_size = static_cast<int>(weights.cols());
	assert(num_data == GetBatchSize());
	assert(data_size == GetOutputSize());

	bool has_weighted_input_layer = HasTrainInputWeights();
	if (has_weighted_input_layer)
	{
		tNNData* input_data = GetTrainInputDataW();
		for (int i = 0; i < num_data; ++i)
		{
			auto curr_data = weights.row(i);
			for (int j = 0; j < data_size; ++j)
			{
				double val = curr_data[j];
				input_data[i * data_size + j] = val;
			}
		}
	}
}

void cNeuralNet::CopyGrad(const cNeuralNet& other)
{
	assert(HasSolver());
	assert(other.HasSolver());
	auto other_net = other.GetTrainNet();
	auto this_net = GetTrainNet();
	
	auto other_params = other_net->learnable_params();
	auto this_params = this_net->learnable_params();
	assert(other_params.size() == this_params.size());

	for (size_t i = 0; i < this_params.size(); ++i)
	{
		auto other_blob = other_params[i];
		auto this_blob = this_params[i];
		assert(other_blob->count() == this_blob->count());

		auto other_diff = GetBlobDiff(other_blob);
		auto this_diff = GetBlobDiffMutable(this_blob);
		for (int j = 0; j < this_blob->count(); ++j)
		{
			this_diff[j] = other_diff[j];
		}
	}
}

const cNeuralNet::tNNData* cNeuralNet::GetBlobData(const caffe::Blob<tNNData>* blob)
{
//#if defined(CPU_ONLY)
	return blob->cpu_data();
//#else
//	return blob->gpu_data();
//#endif
}

cNeuralNet::tNNData* cNeuralNet::GetBlobDataMutable(caffe::Blob<tNNData>* blob)
{
//#if defined(CPU_ONLY)
	return blob->mutable_cpu_data();
//#else
//	return blob->mutable_gpu_data();
//#endif
}

const cNeuralNet::tNNData* cNeuralNet::GetBlobDiff(const caffe::Blob<tNNData>* blob)
{
//#if defined(CPU_ONLY)
	return blob->cpu_diff();
//#else
//	return blob->gpu_diff();
//#endif
}

cNeuralNet::tNNData* cNeuralNet::GetBlobDiffMutable(caffe::Blob<tNNData>* blob)
{
//#if defined(CPU_ONLY)
	return blob->mutable_cpu_diff();
//#else
//	return blob->mutable_gpu_diff();
//#endif
}

bool cNeuralNet::ValidOffsetScale() const
{
	return mInputOffset.size() > 0 && mInputScale.size() > 0
		&& mOutputOffset.size() > 0 && mOutputScale.size() > 0;
}

void cNeuralNet::InitOffsetScale()
{
	int input_size = GetInputSize();
	mInputOffset = Eigen::VectorXd::Zero(input_size);
	mInputScale = Eigen::VectorXd::Ones(input_size);

	int output_size = GetOutputSize();
	mOutputOffset = Eigen::VectorXd::Zero(output_size);
	mOutputScale = Eigen::VectorXd::Ones(output_size);
}

void cNeuralNet::InitTrainNet()
{
	int batch_size = GetBatchSize();
	int output_size = GetOutputSize();
	Eigen::MatrixXd w_init = Eigen::MatrixXd::Ones(batch_size, output_size);
	SetSolverErrWeights(w_init);
}

void cNeuralNet::FetchOutput(const std::vector<caffe::Blob<tNNData>*>& results_arr, Eigen::VectorXd& out_y) const
{
	const caffe::Blob<tNNData>* result = results_arr[results_arr.size() - 1];
	const tNNData* result_data = GetBlobData(result);

	const int output_size = GetOutputSize();
	assert(result->count() == output_size);
	out_y.resize(output_size);

	for (int i = 0; i < output_size; ++i)
	{
		out_y[i] = result_data[i];
	}

	UnnormalizeOutput(out_y);
}

void cNeuralNet::FetchInput(Eigen::VectorXd& out_x) const
{
	const auto& input_blob = mNet->input_blobs()[gXBlobIdx];
	const tNNData* blob_data = GetBlobData(input_blob);

	int input_size = GetInputSize();
	assert(input_blob->count() == input_size);
	out_x.resize(input_size);

	for (int i = 0; i < input_size; ++i)
	{
		out_x[i] = blob_data[i];
	}

	UnnormalizeInput(out_x);
}

void cNeuralNet::NormalizeInput(Eigen::MatrixXd& X) const
{
	if (ValidOffsetScale())
	{
		for (int i = 0; i < X.rows(); ++i)
		{
			auto curr_row = X.row(i);
			curr_row += mInputOffset;
			curr_row = curr_row.cwiseProduct(mInputScale);
		}
	}
}

void cNeuralNet::NormalizeInput(Eigen::VectorXd& x) const
{
	if (ValidOffsetScale())
	{
		assert(x.size() == mInputOffset.size());
		assert(x.size() == mInputScale.size());
		x += mInputOffset;
		x = x.cwiseProduct(mInputScale);
	}
}

void cNeuralNet::NormalizeInputDiff(Eigen::VectorXd& x_diff) const
{
	if (ValidOffsetScale())
	{
		assert(x_diff.size() == mInputScale.size());
		x_diff = x_diff.cwiseProduct(mInputScale);
	}
}

void cNeuralNet::UnnormalizeInput(Eigen::VectorXd& x) const
{
	if (ValidOffsetScale())
	{
		assert(x.size() == mInputScale.size());
		x = x.cwiseQuotient(mInputScale);
		x -= mInputOffset;
	}
}

void cNeuralNet::UnnormalizeInputDiff(Eigen::VectorXd& x_diff) const
{
	if (ValidOffsetScale())
	{
		assert(x_diff.size() == mInputScale.size());
		x_diff = x_diff.cwiseQuotient(mInputScale);
	}
}

void cNeuralNet::NormalizeOutput(Eigen::VectorXd& y) const
{
	if (ValidOffsetScale())
	{
		assert(y.size() == mOutputOffset.size());
		assert(y.size() == mOutputScale.size());
		y += mOutputOffset;
		y = y.cwiseProduct(mOutputScale);
	}
}

void cNeuralNet::UnnormalizeOutput(Eigen::VectorXd& y) const
{
	if (ValidOffsetScale())
	{
		assert(y.size() == mOutputOffset.size());
		assert(y.size() == mOutputScale.size());
		y = y.cwiseQuotient(mOutputScale);
		y -= mOutputOffset;
	}
}

void cNeuralNet::NormalizeOutputDiff(Eigen::VectorXd& y_diff) const
{
	if (ValidOffsetScale())
	{
		assert(y_diff.size() == mOutputScale.size());
		y_diff = y_diff.cwiseProduct(mOutputScale);
	}
}

void cNeuralNet::UnnormalizeOutputDiff(Eigen::VectorXd& y_diff) const
{
	if (ValidOffsetScale())
	{
		assert(y_diff.size() == mOutputScale.size());
		y_diff = y_diff.cwiseQuotient(mOutputScale);
	}
}

boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> cNeuralNet::GetTrainNet() const
{
	if (HasSolver())
	{
		return mSolver->GetNet();
	}
	return nullptr;
}

const std::vector<caffe::Blob<cNeuralNet::tNNData>*>* cNeuralNet::GetTrainInputs() const
{
	if (HasSolver())
	{
		auto train_net = GetTrainNet();
		const std::string& data_layer_name = GetInputLayerName();
		auto input_blobs = &(train_net->input_blobs());
		return input_blobs;
	}
	return nullptr;
}

void cNeuralNet::LoadTrainData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y)
{
	int batch_size = GetBatchSize();
	int num_batches = static_cast<int>(X.rows()) / batch_size;
	assert(num_batches == 1);

	int num_data = batch_size;
	int data_dim = static_cast<int>(X.cols());
	int label_dim = static_cast<int>(Y.cols());
	assert(data_dim == GetProblemXSize());
	assert(label_dim == GetProblemYSize());

	tNNData* x_data = GetTrainInputDataX();
	tNNData* y_data = GetTrainInputDataY();

	bool valid_offset_scale = ValidOffsetScale();
	for (int i = 0; i < num_data; ++i)
	{
		auto curr_data = X.row(i);
		auto curr_label = Y.row(i);

		for (int j = 0; j < data_dim; ++j)
		{
			double val = curr_data[j];
			if (valid_offset_scale)
			{
				val = mInputScale[j] * (val + mInputOffset[j]);
			}
			x_data[i * data_dim + j] = val;
		}

		for (int j = 0; j < label_dim; ++j)
		{
			double val = curr_label[j];
			if (valid_offset_scale)
			{
				val = mOutputScale[j] * (val + mOutputOffset[j]);
			}
			y_data[i * label_dim + j] = val;
		}
	}
}

cNeuralNet::tNNData* cNeuralNet::GetInputDataX() const
{
	const std::vector<caffe::Blob<tNNData>*>& input_blobs = mNet->input_blobs();
	const auto& x_blob = input_blobs[gXBlobIdx];
	tNNData* x_data = GetBlobDataMutable(x_blob);

	const int input_size = GetInputSize();
	assert(input_size == x_blob->count());

	return x_data;
}

cNeuralNet::tNNData* cNeuralNet::GetTrainInputDataX() const
{
	tNNData* x_data = nullptr;
	const auto* input_blobs = GetTrainInputs();
	if (input_blobs != nullptr)
	{
		const auto& x_blob = (*input_blobs)[gXBlobIdx];
		x_data = GetBlobDataMutable(x_blob);

		const int input_size = GetInputSize();
		const int batch_size = GetBatchSize();
		assert(input_size * batch_size == x_blob->count());
	}
	return x_data;
}

cNeuralNet::tNNData* cNeuralNet::GetTrainInputDataY() const
{
	tNNData* y_data = nullptr;
	const auto* input_blobs = GetTrainInputs();
	if (input_blobs != nullptr)
	{
		const auto& y_blob = (*input_blobs)[gYBlobIdx];
		y_data = GetBlobDataMutable(y_blob);

		const int output_size = GetOutputSize();
		const int batch_size = GetBatchSize();
		assert(output_size * batch_size == y_blob->count());
	}
	return y_data;
}

cNeuralNet::tNNData* cNeuralNet::GetTrainInputDataW() const
{
	tNNData* w_data = nullptr;
	const auto* input_blobs = GetTrainInputs();
	if (input_blobs != nullptr && HasTrainInputWeights())
	{
		const auto& w_blob = (*input_blobs)[gWBlobIdx];
		w_data = GetBlobDataMutable(w_blob);

		const int output_size = GetOutputSize();
		const int batch_size = GetBatchSize();
		assert(output_size * batch_size == w_blob->count());
	}
	return w_data;
}

bool cNeuralNet::WriteData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, const std::string& out_file)
{
	bool succ = true;
	int num_data = static_cast<int>(X.rows());
	assert(num_data = static_cast<int>(Y.rows()));
	int x_size = static_cast<int>(X.cols());
	int y_size = static_cast<int>(Y.cols());

	std::vector<float> x_data(num_data * x_size);
	std::vector<float> y_data(num_data * y_size);

	for (int i = 0; i < num_data; ++i)
	{
		const auto& curr_x = X.row(i);
		const auto& curr_y = Y.row(i);

		for (int j = 0; j < x_size; ++j)
		{
			x_data[i * x_size + j] = static_cast<float>(curr_x(j));
		}

		for (int j = 0; j < y_size; ++j)
		{
			y_data[i * y_size + j] = static_cast<float>(curr_y(j));
		}
	}

	const int rank = 4;
	const hsize_t x_dims[rank] = { num_data, 1, 1, x_size };
	const hsize_t y_dims[rank] = { num_data, 1, 1, y_size };

	hid_t file_hid = H5Fcreate(out_file.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT,
								H5P_DEFAULT);
	herr_t status = H5LTmake_dataset_float(file_hid, "data", rank, x_dims, x_data.data());
	if (status == 0)
	{
		status = H5LTmake_dataset_float(file_hid, "label", rank, y_dims, y_data.data());
	}

	if (status != 0)
	{
		succ = false;
	}

	status = H5Fclose(file_hid);
	succ &= (status == 0);
	return succ;
}

std::string cNeuralNet::GetOffsetScaleFile(const std::string& model_file) const
{
	std::string scale_file = model_file;
	scale_file = cFileUtil::RemoveExtension(scale_file);
	scale_file += "_scale.txt";
	return scale_file;
}

void cNeuralNet::WriteOffsetScale(const std::string& norm_file) const
{
	FILE* f = cFileUtil::OpenFile(norm_file, "w");

	if (f != nullptr)
	{
		std::string input_offset_json = cJsonUtil::BuildVectorJson(mInputOffset);
		std::string input_scale_json = cJsonUtil::BuildVectorJson(mInputScale);
		std::string output_offset_json = cJsonUtil::BuildVectorJson(mOutputOffset);
		std::string output_scale_json = cJsonUtil::BuildVectorJson(mOutputScale);

		fprintf(f, "{\n\"%s\": %s,\n\"%s\": %s,\n\"%s\": %s,\n\"%s\": %s\n}", 
			gInputOffsetKey.c_str(), input_offset_json.c_str(),
			gInputScaleKey.c_str(), input_scale_json.c_str(),
			gOutputOffsetKey.c_str(), output_offset_json.c_str(),
			gOutputScaleKey.c_str(), output_scale_json.c_str());

		cFileUtil::CloseFile(f);
	}
	else
	{
		printf("Failed to write offset and scale to %s\n", norm_file.c_str());
	}
}

const std::string& cNeuralNet::GetInputLayerName() const
{
	return gInputLayerName;
}

const std::string& cNeuralNet::GetOutputLayerName() const
{
	return gOutputLayerName;
}


/////////////////////////////
// Caffe Net Wrapper
/////////////////////////////

cNeuralNet::cCaffeNetWrapper::cCaffeNetWrapper(const std::string& net_file, caffe::Phase phase)
	: caffe::Net<tNNData>(net_file, phase)
{
}

cNeuralNet::cCaffeNetWrapper::~cCaffeNetWrapper()
{
}

int cNeuralNet::cCaffeNetWrapper::GetLayerIdx(const std::string& layer_name) const
{
	int idx = layer_names_index_.find(layer_name)->second;
	return idx;
}
