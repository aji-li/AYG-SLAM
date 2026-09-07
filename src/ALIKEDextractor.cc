#include "ALIKEDextractor.h"

#include "aliked.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>
#include <torch/torch.h>

namespace ORB_SLAM2
{
//把 ALIKED 检测结果转成 ORB-SLAM2 可用的 cv::KeyPoint，并按分数筛好
class ALIKEDextractor::Impl
{
public:
    Impl(const std::string& model_name, const std::string& device)
        : modelName(model_name),
          useTensorRT(device == "tensorrt" || device == "trt")
    {
        // Keep the CUDA implementation as a safe fallback for image sizes for
        // which no static TensorRT engine has been exported.
        model = std::make_shared<ALIKED>(model_name, useTensorRT ? "cuda" : device);
        if(useTensorRT)
        {
            std::filesystem::create_directories(engineCacheDirectory());
            loadModelSpecs();
            warmupTensorRT();
        }
    }

    torch::Dict<std::string, torch::Tensor> run(const cv::Mat& image,
                                                const int requestedKeypoints)
    {
        if(!useTensorRT)
            return model->run(image);

        const ModelSpec* spec =
            findModel(image.cols, image.rows, requestedKeypoints);
        if(!spec)
            return model->run(image);

        try
        {
            return runTensorRT(image, *spec);
        }
        catch(const std::exception& error)
        {
            std::cerr << "[ALIKED TensorRT] fallback to LibTorch: "
                      << error.what() << std::endl;
            return model->run(image);
        }
    }

    std::shared_ptr<ALIKED> model;

private:
    struct ModelSpec
    {
        int width;
        int height;
        int topK;
        std::string filename;
    };

    static Ort::Env& environment()
    {
        static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "aliked_tensorrt");
        return env;
    }

    static std::string modelDirectory()
    {
        return std::string(ALIKED_MODELS_DIR) + "/tensorrt";
    }

    static std::string engineCacheDirectory()
    {
        return modelDirectory() + "/cache";
    }

    void loadModelSpecs()
    {
        const std::regex pattern(
            R"(^(.+)-([0-9]+)x([0-9]+)-k([0-9]+)\.onnx$)");
        for(const auto& entry :
            std::filesystem::directory_iterator(modelDirectory()))
        {
            if(!entry.is_regular_file())
                continue;
            std::smatch match;
            const std::string filename = entry.path().filename().string();
            if(!std::regex_match(filename, match, pattern) ||
               match[1].str() != modelName)
                continue;
            modelSpecs.push_back({
                std::stoi(match[2].str()),
                std::stoi(match[3].str()),
                std::stoi(match[4].str()),
                filename});
        }
        std::sort(modelSpecs.begin(), modelSpecs.end(),
                  [](const ModelSpec& lhs, const ModelSpec& rhs) {
                      return lhs.topK < rhs.topK;
                  });
        std::cout << "[ALIKED TensorRT] discovered " << modelSpecs.size()
                  << " static models for " << modelName << std::endl;
    }

    const ModelSpec* findModel(const int width,
                               const int height,
                               const int requestedKeypoints) const
    {
        const ModelSpec* largest = nullptr;
        for(const ModelSpec& spec : modelSpecs)
        {
            if(spec.width != width || spec.height != height)
                continue;
            largest = &spec;
            if(spec.topK >= requestedKeypoints)
                return &spec;
        }
        return largest;
    }

    void warmupTensorRT()
    {
        static const std::array<std::pair<int, int>, 3> sizes{{
            {608, 448}, {501, 368}, {412, 301}}};
        try
        {
            for(const auto& size : sizes)
            {
                const int requested =
                    size.first == 608 ? 792 : (size.first == 501 ? 660 : 548);
                const ModelSpec* spec =
                    findModel(size.first, size.second, requested);
                if(!spec)
                    continue;
                cv::Mat image(spec->height, spec->width, CV_8UC1, cv::Scalar(0));
                (void)runTensorRT(image, *spec);
            }
            std::cout << "[ALIKED TensorRT] warmup complete" << std::endl;
        }
        catch(const std::exception& error)
        {
            std::cerr << "[ALIKED TensorRT] warmup failed, runtime fallback enabled: "
                      << error.what() << std::endl;
        }
    }

    Ort::Session& sessionFor(const ModelSpec& spec)
    {
        std::lock_guard<std::mutex> lock(sessionMutex);
        // Provider registration uses ORT's default logger, so the process-wide
        // environment must exist before providers are appended.
        (void)environment();
        auto found = sessions.find(spec.filename);
        if(found != sessions.end())
            return *found->second;

        const std::string modelPath = modelDirectory() + "/" + spec.filename;
        if(!std::filesystem::exists(modelPath))
            throw std::runtime_error("missing model " + modelPath);

        Ort::SessionOptions options;
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        options.SetIntraOpNumThreads(1);

        OrtTensorRTProviderOptions trtOptions{};
        trtOptions.device_id = 0;
        trtOptions.trt_max_partition_iterations = 1000;
        trtOptions.trt_min_subgraph_size = 1;
        trtOptions.trt_max_workspace_size = 2ULL * 1024ULL * 1024ULL * 1024ULL;
        trtOptions.trt_fp16_enable = 1;
        trtOptions.trt_engine_cache_enable = 1;
        const std::string cachePath = engineCacheDirectory();
        trtOptions.trt_engine_cache_path = cachePath.c_str();
        options.AppendExecutionProvider_TensorRT(trtOptions);

        OrtCUDAProviderOptions cudaOptions{};
        cudaOptions.device_id = 0;
        options.AppendExecutionProvider_CUDA(cudaOptions);

        auto session =
            std::make_unique<Ort::Session>(environment(), modelPath.c_str(), options);
        Ort::Session& result = *session;
        sessions.emplace(spec.filename, std::move(session));
        std::cout << "[ALIKED TensorRT] loaded " << modelPath
                  << " input=" << spec.width << "x" << spec.height
                  << " fp16=1" << std::endl;
        return result;
    }

    torch::Dict<std::string, torch::Tensor>
    runTensorRT(const cv::Mat& image, const ModelSpec& spec)
    {
        if(image.depth() != CV_8U ||
           (image.channels() != 1 && image.channels() != 3))
            throw std::invalid_argument("expected 8-bit grayscale or BGR input");

        std::vector<float> input(
            static_cast<std::size_t>(3) * spec.height * spec.width);
        const int channels = image.channels();
        const std::size_t plane =
            static_cast<std::size_t>(spec.height) * spec.width;
        for(int y = 0; y < spec.height; ++y)
        {
            const unsigned char* row = image.ptr<unsigned char>(y);
            for(int x = 0; x < spec.width; ++x)
            {
                const std::size_t offset =
                    static_cast<std::size_t>(y) * spec.width + x;
                if(channels == 1)
                {
                    const float value = row[x] / 255.0f;
                    input[offset] = value;
                    input[plane + offset] = value;
                    input[2 * plane + offset] = value;
                }
                else
                {
                    input[offset] = row[3 * x + 2] / 255.0f;
                    input[plane + offset] = row[3 * x + 1] / 255.0f;
                    input[2 * plane + offset] = row[3 * x] / 255.0f;
                }
            }
        }

        const std::array<int64_t, 4> inputShape{
            1, 3, spec.height, spec.width};
        auto memoryInfo =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo, input.data(), input.size(),
            inputShape.data(), inputShape.size());

        const char* inputNames[] = {"image"};
        const char* outputNames[] = {"keypoints", "descriptors", "scores"};
        auto outputs = sessionFor(spec).Run(
            Ort::RunOptions{nullptr},
            inputNames, &inputTensor, 1,
            outputNames, 3);

        auto toTensor = [](Ort::Value& value) {
            const auto shape =
                value.GetTensorTypeAndShapeInfo().GetShape();
            int64_t count = 1;
            for(const int64_t dimension : shape)
                count *= dimension;
            return torch::from_blob(
                       value.GetTensorMutableData<float>(),
                       shape,
                       torch::TensorOptions().dtype(torch::kFloat32))
                .clone();
        };

        torch::Dict<std::string, torch::Tensor> result;
        result.insert("keypoints", toTensor(outputs[0]));
        result.insert("descriptors", toTensor(outputs[1]));
        result.insert("scores", toTensor(outputs[2]));
        return result;
    }

    std::string modelName;
    bool useTensorRT = false;
    std::mutex sessionMutex;
    std::vector<ModelSpec> modelSpecs;
    std::unordered_map<std::string, std::unique_ptr<Ort::Session>> sessions;
};

ALIKEDextractor::ALIKEDextractor(const std::string& model_name,
                                 const std::string& device)
    : mpImpl(new Impl(model_name, device))
{
}

ALIKEDextractor::~ALIKEDextractor() = default;

bool ALIKEDextractor::Detect(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints,
                             int max_num) const
{
    cv::Mat descriptors;
    return DetectAndCompute(image, keypoints, descriptors, max_num);
}

} // namespace ORB_SLAM2
