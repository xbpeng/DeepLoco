#include <vector>

#include "caffe/layers/weighted_euclidean_loss_layer.hpp"
#include "caffe/util/math_functions.hpp"

namespace caffe {

template <typename Dtype>
void WeightedEuclideanLossLayer<Dtype>::Forward_gpu(const vector<Blob<Dtype>*>& bottom,
    const vector<Blob<Dtype>*>& top) {
  int count = bottom[0]->count();
  caffe_gpu_sub(
      count,
      bottom[0]->gpu_data(),
      bottom[1]->gpu_data(),
      diff_.mutable_gpu_data());
  caffe_gpu_mul(count, diff_.gpu_data(), bottom[2]->gpu_data(), weighted_diff_.mutable_gpu_data());

  Dtype dot;
  caffe_gpu_dot(count, diff_.gpu_data(), weighted_diff_.gpu_data(), &dot);
  Dtype loss = dot / bottom[0]->num() / Dtype(2);
  top[0]->mutable_cpu_data()[0] = loss;
}

template <typename Dtype>
void WeightedEuclideanLossLayer<Dtype>::Backward_gpu(const vector<Blob<Dtype>*>& top,
    const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom) {
  for (int i = 0; i < 2; ++i) {
    if (propagate_down[i]) {
      const Dtype sign = (i == 0) ? 1 : -1;
      const Dtype alpha = sign * top[0]->cpu_diff()[0] / bottom[i]->num();
      caffe_gpu_axpby(
          bottom[i]->count(),              // count
          alpha,                           // alpha
          weighted_diff_.gpu_data(),       // a
          Dtype(0),                        // beta
          bottom[i]->mutable_gpu_diff());  // b
    }
  }

  {
	  int i = 2;
	  if (propagate_down[i]) {
		  const Dtype alpha = top[0]->cpu_diff()[0] / (bottom[i]->num() * Dtype(2));
		  caffe_gpu_mul(bottom[i]->count(), diff_.gpu_data(), diff_.gpu_data(), bottom[i]->mutable_gpu_diff());
		  caffe_gpu_scal(bottom[i]->count(), alpha, bottom[i]->mutable_gpu_diff());
	  }
  }
}

INSTANTIATE_LAYER_GPU_FUNCS(WeightedEuclideanLossLayer);

}  // namespace caffe
