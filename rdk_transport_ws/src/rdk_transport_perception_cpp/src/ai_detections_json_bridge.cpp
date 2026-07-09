#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {

const std::array<const char *, 80> kCocoNames = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
  "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
  "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
  "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
  "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
  "hair drier", "toothbrush"};

std::unordered_map<std::string, int> MakeClassIds()
{
  std::unordered_map<std::string, int> out;
  for (size_t i = 0; i < kCocoNames.size(); ++i) {
    out.emplace(kCocoNames[i], static_cast<int>(i));
  }
  return out;
}

std::string JsonEscape(const std::string & value)
{
  std::ostringstream oss;
  for (const char ch : value) {
    switch (ch) {
      case '\\': oss << "\\\\"; break;
      case '"': oss << "\\\""; break;
      case '\b': oss << "\\b"; break;
      case '\f': oss << "\\f"; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(ch));
        } else {
          oss << ch;
        }
    }
  }
  return oss.str();
}

std::string JsonString(const std::string & value)
{
  return "\"" + JsonEscape(value) + "\"";
}

}  // namespace

class AiDetectionsJsonBridge : public rclcpp::Node
{
public:
  AiDetectionsJsonBridge()
  : Node("ai_detections_json_bridge"), class_ids_(MakeClassIds())
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/hobot_dnn_detection");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/detections");
    model_path_ = declare_parameter<std::string>(
      "model_path", "/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin");
    score_threshold_ = declare_parameter<double>("score_threshold", 0.35);
    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    target_classes_ = declare_parameter<std::vector<std::string>>(
      "target_classes", {"person", "truck", "chair", "dining table"});

    for (const auto & name : target_classes_) {
      target_class_set_.insert(name);
    }

    pub_ = create_publisher<std_msgs::msg::String>(output_topic, 10);
    sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&AiDetectionsJsonBridge::OnTargets, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "AI detections bridge ready: %s -> %s, image=%dx%d",
      input_topic.c_str(), output_topic.c_str(), image_width_, image_height_);
  }

private:
  void OnTargets(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{";
    json << "\"header\":{\"stamp\":{\"sec\":" << msg->header.stamp.sec
         << ",\"nanosec\":" << msg->header.stamp.nanosec << "},"
         << "\"frame_id\":" << JsonString(msg->header.frame_id) << "},";
    json << "\"image_width\":" << image_width_ << ",";
    json << "\"image_height\":" << image_height_ << ",";
    json << "\"model_path\":" << JsonString(model_path_) << ",";
    WriteClasses(json);
    WriteTargetClasses(json);
    json << "\"inference_ms\":" << InferenceMs(*msg) << ",";
    json << "\"detections\":[";

    bool first = true;
    for (const auto & target : msg->targets) {
      const std::string class_name = target.type.empty() ? "unknown" : target.type;
      const int class_id = ClassId(class_name);
      const bool is_target = target_class_set_.count(class_name) > 0;
      for (const auto & roi : target.rois) {
        if (roi.confidence < score_threshold_) {
          continue;
        }
        if (!first) {
          json << ",";
        }
        first = false;
        const auto & rect = roi.rect;
        const double x1 = Clamp(rect.x_offset, 0, image_width_);
        const double y1 = Clamp(rect.y_offset, 0, image_height_);
        const double x2 = Clamp(rect.x_offset + rect.width, 0, image_width_);
        const double y2 = Clamp(rect.y_offset + rect.height, 0, image_height_);
        json << "{";
        json << "\"class_id\":" << class_id << ",";
        json << "\"class_name\":" << JsonString(class_name) << ",";
        json << "\"score\":" << roi.confidence << ",";
        json << "\"is_target_class\":" << (is_target ? "true" : "false") << ",";
        json << "\"bbox\":{";
        json << "\"x1\":" << x1 << ",\"y1\":" << y1
             << ",\"x2\":" << x2 << ",\"y2\":" << y2
             << ",\"cx\":" << ((x1 + x2) * 0.5)
             << ",\"cy\":" << ((y1 + y2) * 0.5)
             << ",\"w\":" << std::max(0.0, x2 - x1)
             << ",\"h\":" << std::max(0.0, y2 - y1) << "}";
        json << "}";
      }
    }

    json << "]}";
    std_msgs::msg::String out;
    out.data = json.str();
    pub_->publish(out);
  }

  static double Clamp(uint32_t value, int low, int high)
  {
    return static_cast<double>(std::min(std::max(static_cast<int>(value), low), high));
  }

  double InferenceMs(const ai_msgs::msg::PerceptionTargets & msg) const
  {
    for (const auto & perf : msg.perfs) {
      if (perf.time_ms_duration >= 0.0) {
        return perf.time_ms_duration;
      }
    }
    return 0.0;
  }

  int ClassId(const std::string & class_name) const
  {
    const auto it = class_ids_.find(class_name);
    return it == class_ids_.end() ? -1 : it->second;
  }

  void WriteClasses(std::ostringstream & json) const
  {
    json << "\"classes\":[";
    for (size_t i = 0; i < kCocoNames.size(); ++i) {
      if (i > 0) {
        json << ",";
      }
      json << JsonString(kCocoNames[i]);
    }
    json << "],";
  }

  void WriteTargetClasses(std::ostringstream & json) const
  {
    json << "\"target_classes\":[";
    for (size_t i = 0; i < target_classes_.size(); ++i) {
      if (i > 0) {
        json << ",";
      }
      json << JsonString(target_classes_[i]);
    }
    json << "],";
  }

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string model_path_;
  double score_threshold_;
  int image_width_;
  int image_height_;
  std::vector<std::string> target_classes_;
  std::unordered_set<std::string> target_class_set_;
  std::unordered_map<std::string, int> class_ids_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AiDetectionsJsonBridge>());
  rclcpp::shutdown();
  return 0;
}
