#ifndef CAFFE_WEIGHTED_EUCLIDEAN_LOSS_LAYER_HPP_
#define CAFFE_WEIGHTED_EUCLIDEAN_LOSS_LAYER_HPP_

#include "caffe/layers/euclidean_loss_layer.hpp"

namespace caffe {

template <typename Dtype>
class WeightedEuclideanLossLayer : public EuclideanLossLayer<Dtype> {
 public:
  explicit WeightedEuclideanLossLayer(const LayerParameter& param)
      : EuclideanLossLayer<Dtype>(param), weighted_diff_() {}
  virtual void Reshape(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);

  virtual inline int ExactNumBottomBlobs() const { return 3; }
  virtual inline const char* type() const { return "WeightedEuclideanLoss"; }
 

 protected:
  /// @copydoc EuclideanLossLayer
  virtual void Forward_cpu(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
  virtual void Forward_gpu(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);

  virtual void Backward_cpu(const vector<Blob<Dtype>*>& top,
      const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);
  virtual void Backward_gpu(const vector<Blob<Dtype>*>& top,
      const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);

  Blob<Dtype> weighted_diff_;
};

}  // namespace caffe

#endif  // CAFFE_WEIGHTED_EUCLIDEAN_LOSS_LAYER_HPP_
