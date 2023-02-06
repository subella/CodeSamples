// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    keypoint_detector.cpp
 * @author  Samuel Ubellacker
 */
//-----------------------------------------------------------------------------

#include <target_pose_estimating/keypoint_detector.hpp>

namespace sdrone
{

KeypointDetector::
KeypointDetector()
{
  is_initialized_ = false;
};

KeypointDetector::
KeypointDetector(const std::string& model_file_name)
{
  init(model_file_name);
};

void KeypointDetector::
init(const std::string& model_file_name)
{
  torch::jit::getBailoutDepth() = 1;
  torch::autograd::AutoGradMode guard(false);
  module = torch::jit::load(model_file_name);
  assert(module.buffers().size() > 0);
  is_initialized_ = true;
};

bool KeypointDetector::
DetectKeypoints(cv::Mat& raw_img, torch::Tensor& keypoints)
{
    if (!is_initialized_)
        return 0;

    cv::Mat preproc_img;
    PreprocessImage(raw_img, preproc_img);

    torch::Tensor raw_keypoints;
    ForwardPass(preproc_img, raw_keypoints);

    if (!raw_keypoints.numel())
    {
        return 0;
    }

    PostprocessKeypoints(raw_keypoints, keypoints);
    return 1;  
}


c10::IValue KeypointDetector::
GetTracingInputs(cv::Mat& img, c10::Device device) 
{
  const int height = img.rows;
  const int width = img.cols;
  const int channels = 3;

  auto input = torch::from_blob(img.data, {height, width, channels}, torch::kUInt8);
  // HWC to CHW
  input = input.to(device, torch::kFloat).permute({2, 0, 1}).contiguous();
  return input;
}

void KeypointDetector::
PreprocessImage(cv::Mat& raw_img, cv::Mat& preproc_img)
{
    // TODO: No preprocessing is needed for this resolution, but we
    // will have to port the python code to support all resolutions.
    preproc_img = raw_img;
}

void KeypointDetector::
ForwardPass(cv::Mat& preproc_img, torch::Tensor& raw_keypoints)
{
    //TODO don't recreate device each callback
    auto device = (*std::begin(module.buffers())).device();
    auto inputs = GetTracingInputs(preproc_img, device);
    auto output = module.forward({inputs});
    auto outputs = output.toTuple()->elements();
    raw_keypoints = outputs[3].toTensor();

    if (device.is_cuda())
        c10::cuda::getCurrentCUDAStream().synchronize();
}

void KeypointDetector::
PostprocessKeypoints(torch::Tensor& raw_keypoints, torch::Tensor& postproc_keypoints)
{
    // TODO: No postprocessing is needed for this resolution, but we
    // will have to port the python code to support all resolutions.
    postproc_keypoints = raw_keypoints;
}

void KeypointDetector::
DrawKeypoints(cv::Mat& img, torch::Tensor& keypoints)
{
    for (int i = 0; i < keypoints.sizes()[1]; ++i)
    {
        double px = keypoints[0][i][0].item<int>();
        double py = keypoints[0][i][1].item<int>();
        cv::Point pt = cv::Point(px, py);
        cv::circle(img, pt, 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }
}

}