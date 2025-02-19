#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list_.nDeviceNum);

    while (device_list_.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list_.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed; // Width and height will be set dynamically

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        }

        if (MV_OK == nRet) {
          convert_param_.nWidth = out_frame.stFrameInfo.nWidth;
          convert_param_.nHeight = out_frame.stFrameInfo.nHeight;
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          image_msg_.header.stamp = this->now();
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed!");
          {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
          }
          fail_count_++;
        }

        if(fail_count_ > 0){
          if(reconnect()) {
            RCLCPP_INFO(this->get_logger(), "Reconnected successfully!");
            fail_count_ = 0;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Reconnect failed!");
          }
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  bool reconnect()
  {
    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    while (rclcpp::ok()) {
      {
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &stDeviceList);
      }
      if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Enum devices failed! nRet: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      if (stDeviceList.nDeviceNum > 0) {
        nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[0]);
        if (nRet != MV_OK) {
          RCLCPP_ERROR(this->get_logger(), "Create handle failed! nRet: [%x]", nRet);
          continue;
        }

        nRet = MV_CC_OpenDevice(camera_handle_);
        if (nRet == MV_OK) {
          MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposuretime_);
          MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
          MV_CC_SetIntValueEx(camera_handle_, "Width", width_);
          MV_CC_SetIntValueEx(camera_handle_, "Height", height_);
          MV_CC_StartGrabbing(camera_handle_);
          return true;
        } else {
          MV_CC_DestroyHandle(&camera_handle_);
          camera_handle_ = nullptr;
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "No devices found, retrying...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return false;
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    MVCC_INTVALUE_EX i_w_value;
    MVCC_INTVALUE_EX i_h_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time",f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    exposuretime_ = exposure_time;
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    gain_ = gain;
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    //Width
    param_desc.description = "Width";
    MV_CC_GetIntValueEx(camera_handle_, "Width", &i_w_value);
    param_desc.integer_range[0].from_value = i_w_value.nMin;
    param_desc.integer_range[0].to_value = i_w_value.nMax;
    auto width = this->declare_parameter("width", i_w_value.nCurValue, param_desc);
    width_ = width;
    MV_CC_SetIntValueEx(camera_handle_, "Width", width);
    RCLCPP_INFO(this->get_logger(), "Width: %ld", width);

    //Height
    param_desc.description = "Height";
    MV_CC_GetIntValueEx(camera_handle_, "Height", &i_h_value);
    param_desc.integer_range[0].from_value = i_h_value.nMin;
    param_desc.integer_range[0].to_value = i_h_value.nMax;
    auto height = this->declare_parameter("height", i_h_value.nCurValue, param_desc);
    height_ = height;
    MV_CC_SetIntValueEx(camera_handle_, "Height", height);
    RCLCPP_INFO(this->get_logger(), "Height: %ld", height);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        exposuretime_ = param.as_double();
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposuretime_);
        RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposuretime_);
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        gain_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Gain: %f", gain_);
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "width") {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        width_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Width: %d", width_);
        MV_CC_StopGrabbing(camera_handle_);
        int status = MV_CC_SetIntValueEx(camera_handle_, "Width", width_);
        MV_CC_StartGrabbing(camera_handle_);
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set width, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "height") {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        height_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Height: %d", height_);
        MV_CC_StopGrabbing(camera_handle_);
        int status = MV_CC_SetIntValueEx(camera_handle_, "Height", height_);
        MV_CC_StartGrabbing(camera_handle_);
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set height, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  int nRet = MV_OK;
  void* camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  MV_CC_DEVICE_INFO_LIST device_list_;
  int fail_count_ = 0;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::mutex camera_mutex_;

  float exposuretime_ = 0.0;
  double gain_ = 0;
  int width_ = 0;
  int height_ = 0;

};

}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)