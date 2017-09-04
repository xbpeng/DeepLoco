#include <vector>

#include "caffe/layers/fixed_euclidean_loss_layer.hpp"
#include "caffe/util/math_functions.hpp"

namespace caffe {

template <typename Dtype>
void FixedEuclideanLossLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
    const vector<Blob<Dtype>*>& top) {
  int count = bottom[1]->count();
  caffe_copy(count, bottom[1]->cpu_data(), this->diff_.mutable_cpu_data());
  Dtype dot = caffe_cpu_dot(count, this->diff_.cpu_data(), this->diff_.cpu_data());
  Dtype loss = dot / bottom[1]->num() / Dtype(2);
  top[0]->mutable_cpu_data()[0] = loss;
}

#ifdef CPU_ONLY
STUB_GPU(FixedEuclideanLossLayer);
#endif

INSTANTIATE_CLASS(FixedEuclideanLossLayer);
REGISTER_LAYER_CLASS(FixedEuclideanLoss);

}  // namespace caffe