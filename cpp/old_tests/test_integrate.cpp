// v4l2_isp_pipeline.cpp
// Capture from V4L2, run Algo ONNX + RuleEngine ONNX threads, ISP ONNX in main,
// and save final image to a path provided via CLI.

// Compile: g++ -std=cpp17 -O2 -pthread v4l2_isp_pipeline.cpp -o v4l2_isp_pipeline `pkg-config --cflags --libs opencv4` -lonnxruntime

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// ----------------------------- Utility structs -----------------------------
struct Frame {
    cv::Mat bgr;           // captured frame
    uint64_t seq = 0;      // sequence number
    double analog_gain = 2.0;
    double exposure_time = 0.01;
    double sensor_temp = 40.0;
    float scene_change = 0.0f; // 0 or 1
};

struct RawCoeffs {
    uint64_t seq = 0;
    std::vector<float> wb;        // len 3
    std::vector<float> ccm;       // len 9
    std::vector<float> gamma;     // len N
    std::vector<float> sharpness; // len 1
    std::vector<float> nr;        // len 1
};

struct StabCoeffs {
    uint64_t seq = 0;
    std::vector<float> wb;        // len 3
    std::vector<float> ccm;       // len 9
    std::vector<float> gamma;     // len N
    std::vector<float> sharpness; // len 1
    std::vector<float> nr;        // len 1
};

// Thread-safe queue template
template<typename T>
class TSQueue {
public:
    void push(const T& item) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push(item);
        cv_.notify_one();
    }
    bool pop(T& out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty() || stop_; });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop();
        return true;
    }
    void stop() {
        std::lock_guard<std::mutex> lk(m_);
        stop_ = true;
        cv_.notify_all();
    }
private:
    std::queue<T> q_;
    std::mutex m_;
    std::condition_variable cv_;
    bool stop_ = false;
};

// ----------------------------- V4L2 capture -----------------------------
class V4L2Capture {
public:
    V4L2Capture() = default;
    ~V4L2Capture() { closeDevice(); }

    bool openDevice(const std::string& dev, int width, int height, int fps=30) {
        fd_ = ::open(dev.c_str(), O_RDWR);
        if (fd_ < 0) {
            std::cerr << "Failed to open device: " << dev << std::endl;
            return false;
        }

        v4l2_capability cap{};
        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            std::cerr << "VIDIOC_QUERYCAP failed\n";
            return false;
        }

        // Set format to MJPEG or YUYV; we’ll decode with OpenCV
        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // try MJPEG first
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            // fallback to YUYV
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
                std::cerr << "VIDIOC_S_FMT failed\n";
                return false;
            }
            use_yuyv_ = true;
        } else {
            use_yuyv_ = false;
        }

        // Set FPS
        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = fps;
        ioctl(fd_, VIDIOC_S_PARM, &parm);

        // Request buffers
        v4l2_requestbuffers req{};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            std::cerr << "VIDIOC_REQBUFS failed\n";
            return false;
        }

        buffers_.resize(req.count);
        for (unsigned i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                std::cerr << "VIDIOC_QUERYBUF failed\n";
                return false;
            }
            void* start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (start == MAP_FAILED) {
                std::cerr << "mmap failed\n";
                return false;
            }
            buffers_[i] = {start, buf.length};
        }

        // Queue buffers
        for (unsigned i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                std::cerr << "VIDIOC_QBUF failed\n";
                return false;
            }
        }

        // Start streaming
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            std::cerr << "VIDIOC_STREAMON failed\n";
            return false;
        }

        width_ = fmt.fmt.pix.width;
        height_ = fmt.fmt.pix.height;
        std::cerr << "V4L2 opened: " << dev << ", " << width_ << "x" << height_
                  << " format=" << (use_yuyv_ ? "YUYV" : "MJPEG") << std::endl;
        return true;
    }

    bool readFrame(cv::Mat& out_bgr) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            std::cerr << "VIDIOC_DQBUF failed\n";
            return false;
        }
        void* data = buffers_[buf.index].start;
        size_t len = buf.bytesused;

        // Decode to BGR with OpenCV
        if (!use_yuyv_) {
            // MJPEG -> BGR
            std::vector<uchar> mjpeg((uchar*)data, (uchar*)data + len);
            cv::Mat mjpegMat(mjpeg, true);
            out_bgr = cv::imdecode(mjpegMat, cv::IMREAD_COLOR);
        } else {
            // YUYV -> BGR
            cv::Mat yuyv(height_, width_, CV_8UC2, data);
            cv::cvtColor(yuyv, out_bgr, cv::COLOR_YUV2BGR_YUYV);
        }

        // Re-queue
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF failed\n";
            return false;
        }
        return !out_bgr.empty();
    }

    void closeDevice() {
        if (fd_ >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
            for (auto& b : buffers_) {
                if (b.start) munmap(b.start, b.length);
            }
            ::close(fd_);
            fd_ = -1;
        }
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    struct Buffer { void* start = nullptr; size_t length = 0; };
    int fd_ = -1;
    int width_ = 0, height_ = 0;
    bool use_yuyv_ = false;
    std::vector<Buffer> buffers_;
};

// ----------------------------- ONNX utilities -----------------------------
struct OnnxModel {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "v4l2_isp"};
    Ort::SessionOptions opts;
    std::unique_ptr<Ort::Session> session;

    OnnxModel() {
        opts.SetIntraOpNumThreads(1);
        opts.SetInterOpNumThreads(1);
#ifdef _WIN32
        // CPU EP is default
#else
        // Nothing extra needed for CPU EP
#endif
    }

    bool load(const std::string& path) {
        try {
            session.reset(new Ort::Session(env, path.c_str(), opts));
            return true;
        } catch (const Ort::Exception& e) {
            std::cerr << "Failed to load ONNX: " << path << " error: " << e.what() << std::endl;
            return false;
        }
    }

    std::vector<const char*> inputNames() {
        Ort::AllocatorWithDefaultOptions allocator;
        size_t count = session->GetInputCount();
        std::vector<const char*> names;
        names.reserve(count);
        name_store_in_.clear();
        for (size_t i = 0; i < count; ++i) {
            char* name = session->GetInputNameAllocated(i, allocator).get();
            name_store_in_.emplace_back(name);
            names.push_back(name_store_in_.back().c_str());
        }
        return names;
    }

    std::vector<const char*> outputNames() {
        Ort::AllocatorWithDefaultOptions allocator;
        size_t count = session->GetOutputCount();
        std::vector<const char*> names;
        names.reserve(count);
        name_store_out_.clear();
        for (size_t i = 0; i < count; ++i) {
            char* name = session->GetOutputNameAllocated(i, allocator).get();
            name_store_out_.emplace_back(name);
            names.push_back(name_store_out_.back().c_str());
        }
        return names;
    }

private:
    std::vector<std::string> name_store_in_;
    std::vector<std::string> name_store_out_;
};

// ----------------------------- Global pipeline state -----------------------------
TSQueue<Frame> frame_q;
TSQueue<RawCoeffs> raw_q;
TSQueue<StabCoeffs> stab_q;

std::atomic<bool> stop_all{false};

// Keep previous stabilized coeffs (initial defaults)
StabCoeffs prev_stab{
    0,
    std::vector<float>{1.0f, 1.0f, 1.0f},
    std::vector<float>{1,0,0, 0,1,0, 0,0,1},
    std::vector<float>(256), // gamma initialized to linear
    std::vector<float>{1.0f},
    std::vector<float>{0.5f}
};

// ----------------------------- Algo Thread -----------------------------
void algo_thread_func(OnnxModel& algoModel, int gammaN) {
    auto in_names = algoModel.inputNames();
    auto out_names = algoModel.outputNames();

    Ort::AllocatorWithDefaultOptions alloc;
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    while (!stop_all.load()) {
        Frame fr;
        if (!frame_q.pop(fr)) break;

        // Prepare inputs for Algo ONNX: assume it consumes image as NHWC float32 [1, H, W, 3]
        cv::Mat bgr32f;
        fr.bgr.convertTo(bgr32f, CV_32FC3, 1.0 / 255.0);
        std::vector<int64_t> in_shape{1, fr.bgr.rows, fr.bgr.cols, 3};
        size_t numel = (size_t)in_shape[0]*in_shape[1]*in_shape[2]*in_shape[3];
        std::vector<float> img(numel);
        std::memcpy(img.data(), bgr32f.data, numel * sizeof(float));

        Ort::Value imgTensor = Ort::Value::CreateTensor<float>(memInfo, img.data(), img.size(), in_shape.data(), in_shape.size());

        std::vector<Ort::Value> inputs;
        inputs.push_back(std::move(imgTensor));

        // Run Algo
        auto outputs = algoModel.session->Run(Ort::RunOptions{nullptr}, in_names.data(), inputs.data(), inputs.size(), out_names.data(), out_names.size());

        // Extract raw coeffs from outputs by name convention:
        // raw_wb[3], raw_ccm[9], raw_gamma[N], raw_sharpness[1], raw_nr[1]
        RawCoeffs rc;
        rc.seq = fr.seq;
        rc.wb.resize(3);
        rc.ccm.resize(9);
        rc.gamma.resize(gammaN);
        rc.sharpness.resize(1);
        rc.nr.resize(1);

        auto get_vec = [&](const char* name, std::vector<float>& dst){
            // Find index by name; fallback by position if not found
            int idx = -1;
            for (size_t i = 0; i < out_names.size(); ++i) {
                if (std::string(out_names[i]) == std::string(name)) { idx = (int)i; break; }
            }
            if (idx < 0) idx = 0; // simplistic fallback
            float* data = outputs[idx].GetTensorMutableData<float>();
            Ort::TensorTypeAndShapeInfo ti = outputs[idx].GetTensorTypeAndShapeInfo();
            size_t n = ti.GetElementCount();
            dst.assign(data, data + n);
        };

        // Try using canonical names; adjust if your Algo ONNX differs
        get_vec("wb_gains", rc.wb);
        get_vec("ccm", rc.ccm);
        get_vec("gamma_lut", rc.gamma);
        get_vec("sharpness_strength", rc.sharpness);
        get_vec("nr_sigma", rc.nr);

        raw_q.push(rc);
    }
}

// ----------------------------- RuleEngine Thread -----------------------------
void rule_thread_func(OnnxModel& ruleModel, int gammaN) {
    auto in_names = ruleModel.inputNames();
    auto out_names = ruleModel.outputNames();

    Ort::AllocatorWithDefaultOptions alloc;
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    while (!stop_all.load()) {
        RawCoeffs rc;
        if (!raw_q.pop(rc)) break;

        // Build input tensors for RuleEngine ONNX
        auto make_tensor = [&](const std::vector<float>& vec, const std::vector<int64_t>& shape){
            return Ort::Value::CreateTensor<float>(memInfo, const_cast<float*>(vec.data()), vec.size(), shape.data(), shape.size());
        };
        auto make_scalar = [&](float v){
            std::vector<float> s{v};
            std::vector<int64_t> shape{1};
            return Ort::Value::CreateTensor<float>(memInfo, s.data(), s.size(), shape.data(), shape.size());
        };

        // Raw coeffs
        Ort::Value raw_wb = make_tensor(rc.wb, {3});
        Ort::Value raw_ccm = make_tensor(rc.ccm, {9});
        Ort::Value raw_gamma = make_tensor(rc.gamma, {gammaN});
        Ort::Value raw_sharpness = make_tensor(rc.sharpness, {1});
        Ort::Value raw_nr = make_tensor(rc.nr, {1});

        // Prev coeffs
        Ort::Value prev_wb = make_tensor(prev_stab.wb, {3});
        Ort::Value prev_ccm = make_tensor(prev_stab.ccm, {9});
        Ort::Value prev_gamma = make_tensor(prev_stab.gamma, {gammaN});
        Ort::Value prev_sharpness = make_tensor(prev_stab.sharpness, {1});
        Ort::Value prev_nr = make_tensor(prev_stab.nr, {1});

        // Sensor meta (dummy)
        Ort::Value analog_gain = make_scalar(2.0f);
        Ort::Value exposure_time = make_scalar(0.01f);
        Ort::Value sensor_temp = make_scalar(40.0f);
        Ort::Value scene_change = make_scalar(0.0f);

        // Rule params (runtime tunable)
        Ort::Value alpha_wb = make_scalar(0.2f);
        Ort::Value alpha_ccm = make_scalar(0.2f);
        Ort::Value alpha_gamma = make_scalar(0.2f);
        Ort::Value alpha_sharp = make_scalar(0.2f);
        Ort::Value alpha_nr = make_scalar(0.2f);
        Ort::Value alpha_fast = make_scalar(0.5f);

        Ort::Value wb_step = make_scalar(0.05f);
        Ort::Value sharp_step = make_scalar(0.05f);

        Ort::Value wb_min = make_scalar(0.5f);
        Ort::Value wb_max = make_scalar(2.0f);

        Ort::Value gamma_min = make_scalar(0.0f);
        Ort::Value gamma_max = make_scalar(1.0f);

        Ort::Value ccm_min = make_scalar(-2.0f);
        Ort::Value ccm_max = make_scalar(2.0f);

        Ort::Value nr_min = make_scalar(0.0f);
        Ort::Value nr_max = make_scalar(5.0f);

        // Order must match RuleEngine inputs
        std::vector<Ort::Value> inputs;
        inputs.reserve(in_names.size());
        auto push_by_name = [&](const char* name, Ort::Value&& val){
            // maintains semantic mapping; in production, map exactly by name
            inputs.push_back(std::move(val));
        };

        // Push in canonical order:
        // raw_* , prev_* , sensor_* , alpha_* , steps , clamps
        inputs.push_back(std::move(raw_wb));
        inputs.push_back(std::move(raw_ccm));
        inputs.push_back(std::move(raw_gamma));
        inputs.push_back(std::move(raw_sharpness));
        inputs.push_back(std::move(raw_nr));

        inputs.push_back(std::move(prev_wb));
        inputs.push_back(std::move(prev_ccm));
        inputs.push_back(std::move(prev_gamma));
        inputs.push_back(std::move(prev_sharpness));
        inputs.push_back(std::move(prev_nr));

        inputs.push_back(std::move(analog_gain));
        inputs.push_back(std::move(exposure_time));
        inputs.push_back(std::move(sensor_temp));
        inputs.push_back(std::move(scene_change));

        inputs.push_back(std::move(alpha_wb));
        inputs.push_back(std::move(alpha_ccm));
        inputs.push_back(std::move(alpha_gamma));
        inputs.push_back(std::move(alpha_sharp));
        inputs.push_back(std::move(alpha_nr));
        inputs.push_back(std::move(alpha_fast));

        inputs.push_back(std::move(wb_step));
        inputs.push_back(std::move(sharp_step));

        inputs.push_back(std::move(wb_min));
        inputs.push_back(std::move(wb_max));
        inputs.push_back(std::move(gamma_min));
        inputs.push_back(std::move(gamma_max));
        inputs.push_back(std::move(ccm_min));
        inputs.push_back(std::move(ccm_max));
        inputs.push_back(std::move(nr_min));
        inputs.push_back(std::move(nr_max));

        // Run RuleEngine
        auto outputs = ruleModel.session->Run(Ort::RunOptions{nullptr}, in_names.data(), inputs.data(), inputs.size(), out_names.data(), out_names.size());

        // Collect stabilized outputs: wb_stab, ccm_stab, gamma_stab, sharpness_stab, nr_stab
        StabCoeffs sc;
        sc.seq = rc.seq;
        sc.wb.resize(3);
        sc.ccm.resize(9);
        sc.gamma.resize(gammaN);
        sc.sharpness.resize(1);
        sc.nr.resize(1);

        auto copy_out = [&](const char* name, std::vector<float>& dst){
            int idx = -1;
            for (size_t i = 0; i < out_names.size(); ++i) {
                if (std::string(out_names[i]) == std::string(name)) { idx = (int)i; break; }
            }
            if (idx < 0) idx = 0;
            float* data = outputs[idx].GetTensorMutableData<float>();
            Ort::TensorTypeAndShapeInfo ti = outputs[idx].GetTensorTypeAndShapeInfo();
            size_t n = ti.GetElementCount();
            dst.assign(data, data + n);
        };
        copy_out("wb_stab", sc.wb);
        copy_out("ccm_stab", sc.ccm);
        copy_out("gamma_stab", sc.gamma);
        copy_out("sharpness_stab", sc.sharpness);
        copy_out("nr_stab", sc.nr);

        // Update prev_stab for next frame
        prev_stab = sc;

        // Push to ISP queue
        stab_q.push(sc);
    }
}

// ----------------------------- ISP execution -----------------------------
cv::Mat run_isp(OnnxModel& ispModel, const Frame& fr, const StabCoeffs& sc) {
    auto in_names = ispModel.inputNames();
    auto out_names = ispModel.outputNames();

    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // Prepare pixel input as float32 [1, H, W, 3]
    cv::Mat bgr32f;
    fr.bgr.convertTo(bgr32f, CV_32FC3, 1.0 / 255.0);
    std::vector<int64_t> img_shape{1, fr.bgr.rows, fr.bgr.cols, 3};
    size_t numel = (size_t)img_shape[0]*img_shape[1]*img_shape[2]*img_shape[3];
    std::vector<float> img(numel);
    std::memcpy(img.data(), bgr32f.data, numel * sizeof(float));
    Ort::Value imgTensor = Ort::Value::CreateTensor<float>(memInfo, img.data(), img.size(), img_shape.data(), img_shape.size());

    // Prepare coeff tensors
    auto make_tensor = [&](const std::vector<float>& vec, const std::vector<int64_t>& shape){
        return Ort::Value::CreateTensor<float>(memInfo, const_cast<float*>(vec.data()), vec.size(), shape.data(), shape.size());
    };

    Ort::Value wb = make_tensor(sc.wb, {3});
    Ort::Value ccm = make_tensor(sc.ccm, {9});
    Ort::Value gamma = make_tensor(sc.gamma, {(int64_t)sc.gamma.size()});
    Ort::Value sharp = make_tensor(sc.sharpness, {1});
    Ort::Value nr = make_tensor(sc.nr, {1});

    // Order must match your ISP ONNX inputs: pixels + coeffs
    std::vector<Ort::Value> inputs;
    inputs.push_back(std::move(imgTensor));
    inputs.push_back(std::move(wb));
    inputs.push_back(std::move(ccm));
    inputs.push_back(std::move(gamma));
    inputs.push_back(std::move(sharp));
    inputs.push_back(std::move(nr));

    auto outputs = ispModel.session->Run(Ort::RunOptions{nullptr}, in_names.data(), inputs.data(), inputs.size(), out_names.data(), out_names.size());

    // Assume single output tensor: final image [1, H, W, 3] float32
    float* outData = outputs[0].GetTensorMutableData<float>();
    Ort::TensorTypeAndShapeInfo ti = outputs[0].GetTensorTypeAndShapeInfo();
    auto shape = ti.GetShape();
    if (shape.size() != 4 || shape[3] != 3) {
        std::cerr << "Unexpected ISP output shape\n";
        return cv::Mat();
    }
    int H = (int)shape[1], W = (int)shape[2];
    cv::Mat out32f(H, W, CV_32FC3, outData);
    cv::Mat out8u;
    out32f.convertTo(out8u, CV_8UC3, 255.0);
    return out8u.clone(); // clone because outData is managed by ORT
}

// ----------------------------- Main -----------------------------
int main(int argc, char** argv) {
    std::string device = "/dev/video0";
    std::string algo_path = "algo.onnx";
    std::string rule_path = "ruleengine_full.onnx";
    std::string isp_path = "isp.onnx";
    std::string out_path = "output.png";
    int frames_to_capture = 1;
    int width = 640, height = 480, fps = 30;
    int gammaN = 256;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](int& idx){ return (i+1 < argc) ? std::string(argv[++i]) : ""; };
        if (a == "--device") device = next(i);
        else if (a == "--algo") algo_path = next(i);
        else if (a == "--rule") rule_path = next(i);
        else if (a == "--isp") isp_path = next(i);
        else if (a == "--out") out_path = next(i);
        else if (a == "--frames") frames_to_capture = std::stoi(next(i));
        else if (a == "--size") { std::string s = next(i); auto pos = s.find('x'); width = std::stoi(s.substr(0,pos)); height = std::stoi(s.substr(pos+1)); }
        else if (a == "--fps") fps = std::stoi(next(i));
    }

    // Initialize V4L2
    V4L2Capture cap;
    if (!cap.openDevice(device, width, height, fps)) {
        std::cerr << "Failed to open V4L2 device\n";
        return 1;
    }

    // Load models
    OnnxModel algoModel, ruleModel, ispModel;
    if (!algoModel.load(algo_path)) return 1;
    if (!ruleModel.load(rule_path)) return 1;
    if (!ispModel.load(isp_path)) return 1;

    // Start threads
    std::thread algo_thread(algo_thread_func, std::ref(algoModel), gammaN);
    std::thread rule_thread(rule_thread_func, std::ref(ruleModel), gammaN);

    // Capture frames and hand off to queues
    uint64_t seq = 0;
    cv::Mat final_image;
    for (int f = 0; f < frames_to_capture; ++f) {
        Frame fr;
        fr.seq = seq++;
        if (!cap.readFrame(fr.bgr)) {
            std::cerr << "Failed to read frame\n";
            break;
        }
        // Optionally detect scene change here based on luminance hist; keep 0.0 for now
        frame_q.push(fr);

        // Wait for stabilized coeffs with matching seq (simple approach)
        StabCoeffs sc;
        bool got = false;
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
            if (!stab_q.pop(sc)) break;
            if (sc.seq == fr.seq) { got = true; break; }
        }
        if (!got) {
            std::cerr << "Timeout waiting for stabilized coeffs\n";
            continue;
        }

        // Run ISP ONNX
        final_image = run_isp(ispModel, fr, sc);
    }

    // Stop threads
    stop_all.store(true);
    frame_q.stop();
    raw_q.stop();
    stab_q.stop();
    if (algo_thread.joinable()) algo_thread.join();
    if (rule_thread.joinable()) rule_thread.join();

    // Save final image
    if (!final_image.empty()) {
        if (!cv::imwrite(out_path, final_image)) {
            std::cerr << "Failed to write image to: " << out_path << std::endl;
        } else {
            std::cout << "Saved ISP output to: " << out_path << std::endl;
        }
    } else {
        std::cerr << "No final image to save\n";
    }

    return 0;
}
