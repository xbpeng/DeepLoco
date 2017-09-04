#ifndef CAFFE_FIXED_LOSS_LAYER_HPP_
#define CAFFE_FIXED_LOSS_LAYER_HPP_

#include "caffe/layers/euclidean_loss_layer.hpp"

namespace caffe {

template <typename Dtype>
class FixedEuclideanLossLayer : public EuclideanLossLayer<Dtype> {
 public:
  explicit FixedEuclideanLossLayer(const LayerParameter& param)
      : EuclideanLossLayer<Dtype>(param) {}

  virtual inline const char* type() const { return "FixedEuclideanLoss"; }

 protected:

  virtual void Forward_cpu(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
  virtual void Forward_gpu(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);

  virtual void Backward_gpu(const vector<Blob<Dtype>*>& top,
      const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);
};

}  // namespace caffe

#endif  // CAFFE_FIXED_LOSS_LAYER_HPP_
