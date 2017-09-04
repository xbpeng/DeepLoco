#include <vector>

#include "caffe/layers/weighted_euclidean_loss_layer.hpp"
#include "caffe/util/math_functions.hpp"

namespace caffe {

template <typename Dtype>
void WeightedEuclideanLossLayer<Dtype>::Reshape(
  const vector<Blob<Dtype>*>& bottom, const vector<Blob<Dtype>*>& top) {
	CHECK_EQ(bottom[0]->count(1), bottom[2]->count(1))
		<< "Inputs must have the same dimension.";
	EuclideanLossLayer<Dtype>::Reshape(bottom, top);
	weighted_diff_.ReshapeLike(*bottom[0]);
}

template <typename Dtype>
void WeightedEuclideanLossLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
    const vector<Blob<Dtype>*>& top) {
  int count = bottom[0]->count();
  caffe_sub(
      count,
      bottom[0]->cpu_data(),
      bottom[1]->cpu_data(),
      this->diff_.mutable_cpu_data());
  caffe_mul(count, this->diff_.cpu_data(), bottom[2]->cpu_data(), weighted_diff_.mutable_cpu_data());

  Dtype dot = caffe_cpu_dot(count, this->diff_.cpu_data(), weighted_diff_.cpu_data());
  Dtype loss = dot / bottom[0]->num() / Dtype(2);
  top[0]->mutable_cpu_data()[0] = loss;
}

template <typename Dtype>
void WeightedEuclideanLossLayer<Dtype>::Backward_cpu(const vector<Blob<Dtype>*>& top,
    const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom) {
  for (int i = 0; i < 2; ++i) {
    if (propagate_down[i]) {
      const Dtype sign = (i == 0) ? 1 : -1;
      const Dtype alpha = sign * top[0]->cpu_diff()[0] / bottom[i]->num();
      caffe_cpu_axpby(
          bottom[i]->count(),              // count
          alpha,                           // alpha
		  weighted_diff_.cpu_data(),       // a
          Dtype(0),                        // beta
          bottom[i]->mutable_cpu_diff());  // b
    }
  }

  {
	  int i = 2;
	  if (propagate_down[i]) {
		  const Dtype alpha = top[0]->cpu_diff()[0] / (bottom[i]->num() * Dtype(2));
		  caffe_mul(bottom[i]->count(), this->diff_.cpu_data(), this->diff_.cpu_data(), bottom[i]->mutable_cpu_diff());
		  caffe_scal(bottom[i]->count(), alpha, bottom[i]->mutable_cpu_diff());
	  }
  }
}

#ifdef CPU_ONLY
STUB_GPU(WeightedEuclideanLossLayer);
#endif

INSTANTIATE_CLASS(WeightedEuclideanLossLayer);
REGISTER_LAYER_CLASS(WeightedEuclideanLoss);

}  // namespace caffe