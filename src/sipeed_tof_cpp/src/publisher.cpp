#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"

#include <event2/event.h>
#include <event2/http.h>
#include <event2/buffer.h>
#include <event2/http_struct.h>

#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include "cJSON.h"
#include "process.hpp"

class MaixSenseCamera {
public:
    MaixSenseCamera(const std::string& name,
                    const std::string& host,
                    int port,
                    const std::string& frame_id,
                    ros::NodeHandle& nh_parent)
        : name_(name), host_(host), port_(port), frame_id_(frame_id), nh_(nh_parent, name) {}

    ~MaixSenseCamera() {
        if (conn_) {
            evhttp_connection_free(conn_);
            conn_ = nullptr;
        }
        if (base_) {
            event_base_free(base_);
            base_ = nullptr;
        }
    }

    bool init() {
        ROS_INFO_STREAM("[" << name_ << "] init camera at " << host_ << ":" << port_);

        filter_.reset(new Tof_Filter(320, 240, 7));
        filter_->TemporalFilter_cfg(0.5);
        filter_->set_kernel_size(7);

        base_ = event_base_new();
        if (!base_) {
            ROS_ERROR_STREAM("[" << name_ << "] failed to create event_base");
            return false;
        }

        conn_ = evhttp_connection_base_new(base_, nullptr, host_.c_str(), port_);
        if (!conn_) {
            ROS_ERROR_STREAM("[" << name_ << "] failed to create HTTP connection");
            return false;
        }
        evhttp_connection_set_timeout(conn_, 60);

        if (!request(EVHTTP_REQ_GET, "/getinfo", nullptr, 0)) {
            ROS_ERROR_STREAM("[" << name_ << "] /getinfo failed");
            return false;
        }
        if (response_.size() < sizeof(info_t)) {
            ROS_ERROR_STREAM("[" << name_ << "] /getinfo returned too few bytes: " << response_.size());
            return false;
        }
        info_t info_all;
        std::memcpy(&info_all, response_.data(), sizeof(info_t));
        uvf_parms_ = filter_->parse_info(&info_all);

        if (!request(EVHTTP_REQ_GET, "/get_lut", nullptr, 0)) {
            ROS_ERROR_STREAM("[" << name_ << "] /get_lut failed");
            return false;
        }
        if (response_.size() < 65536) {
            ROS_ERROR_STREAM("[" << name_ << "] /get_lut returned too few bytes: " << response_.size());
            return false;
        }
        filter_->set_lut(response_.data());

        if (!request(EVHTTP_REQ_GET, "/CameraParms.json", nullptr, 0)) {
            ROS_ERROR_STREAM("[" << name_ << "] /CameraParms.json failed");
            return false;
        }
        if (!loadCameraParms()) {
            ROS_ERROR_STREAM("[" << name_ << "] failed to parse CameraParms.json");
            return false;
        }

        all_config_t config;
        std::memset(&config, 0, sizeof(config));
        config.triggermode = 1;   // 0: STOP, 1: AUTO, 2: SINGLE
        config.deepmode = 1;      // 0: 16-bit, 1: 8-bit
        config.deepshift = 255;   // for 8-bit mode
        config.irmode = 1;        // 0: 16-bit, 1: 8-bit
        config.statusmode = 1;    // 0: 16-bit, 1: 2-bit, 2: 8-bit, 3: 1-bit
        config.statusmask = 7;
        config.rgbmode = 1;       // 0: YUV, 1: JPG, 2: None
        config.rgbres = 0;        // 0: 800x600, 1: 1600x1200
        config.expose_time = 0;

        if (!request(EVHTTP_REQ_POST, "/set_cfg", &config, sizeof(config))) {
            ROS_ERROR_STREAM("[" << name_ << "] /set_cfg failed");
            return false;
        }

        pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 10);
        pub_rgb_ = nh_.advertise<sensor_msgs::Image>("rgb", 10);
        pub_depth_ = nh_.advertise<sensor_msgs::Image>("depth", 10);
        pub_intensity_ = nh_.advertise<sensor_msgs::Image>("intensity", 10);
        pub_status_ = nh_.advertise<sensor_msgs::Image>("status", 10);

        // 仅用于判断当前点云数值坐标轴的物理方向：
        // 只发布 x > 0 且 y > 0 且 z > 0 的点，原始 cloud 与 RGB 颜色投影完全不变。
        pub_first_quadrant_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("first_quadrant_cloud", 1);

        ROS_INFO_STREAM("[" << name_ << "] initialized. Topics are under /" << name_ << "/...");
        ROS_INFO_STREAM("[" << name_ << "] quadrant test topic: /" << name_
                        << "/first_quadrant_cloud (x > 0, y > 0, z > 0)");
        return true;
    }

    bool pollAndPublish() {
        if (!request(EVHTTP_REQ_GET, "/getdeep", nullptr, 0)) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[" << name_ << "] /getdeep failed");
            return false;
        }

        stackframe_old_t oldframe = filter_->DecodePkg(response_.data());

        auto data_out1 = filter_->TemporalFilter(reinterpret_cast<uint16_t*>(oldframe.depth));
        auto data_out2 = filter_->SpatialFilter(data_out1, 0);
        auto data_out3 = filter_->FlyingPointFilter(data_out2, 0.03);

        auto status_vec = std::vector<uint16_t>(
            reinterpret_cast<uint16_t*>(oldframe.status),
            reinterpret_cast<uint16_t*>(reinterpret_cast<uint8_t*>(oldframe.status) + sizeof(Image_t)));

        auto tempcali = filter_->TOF_cali(data_out3, status_vec);
        auto colormap_temp = filter_->MapRGB2TOF(
            tempcali,
            std::vector<uint8_t>(oldframe.rgb, oldframe.rgb + sizeof(oldframe.rgb)));

        auto tempcali_ir = filter_->TOF_cali(
            std::vector<uint16_t>(reinterpret_cast<uint16_t*>(oldframe.ir),
                                  reinterpret_cast<uint16_t*>(reinterpret_cast<uint8_t*>(oldframe.ir) + sizeof(Image_t))),
            status_vec);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id_;

        publishImages(oldframe, header);
        publishPointCloud(tempcali, tempcali_ir, colormap_temp, header);
        return true;
    }

private:
    static void httpDoneCb(evhttp_request* req, void* arg) {
        MaixSenseCamera* self = static_cast<MaixSenseCamera*>(arg);
        self->response_.clear();
        self->request_ok_ = false;

        if (req) {
            evbuffer* input = evhttp_request_get_input_buffer(req);
            const size_t len = evbuffer_get_length(input);
            self->response_.resize(len);
            if (len > 0) {
                evbuffer_remove(input, self->response_.data(), len);
            }
            self->request_ok_ = true;
        }

        if (self->base_) {
            event_base_loopbreak(self->base_);
        }
    }

    bool request(evhttp_cmd_type type, const char* url, const void* data, size_t data_size) {
        response_.clear();
        request_ok_ = false;

        evhttp_request* req = evhttp_request_new(&MaixSenseCamera::httpDoneCb, this);
        if (!req) {
            return false;
        }

        evhttp_add_header(evhttp_request_get_output_headers(req), "Host", host_.c_str());
        evhttp_add_header(evhttp_request_get_output_headers(req), "Connection", "keep-alive");

        if (data && data_size > 0) {
            evbuffer_add(evhttp_request_get_output_buffer(req), data, data_size);
        }

        if (evhttp_make_request(conn_, req, type, url) != 0) {
            ROS_ERROR_STREAM("[" << name_ << "] evhttp_make_request failed for " << url);
            return false;
        }

        event_base_dispatch(base_);
        return request_ok_;
    }

    bool loadCameraParms() {
        cJSON* cparms = cJSON_ParseWithLength(
            reinterpret_cast<const char*>(response_.data()), response_.size());
        if (!cparms) {
            return false;
        }

        float R_data[9];
        float T_data[3];
        float RGB_CM_data[9];
        float D_VEC_data[5];

        cJSON* tempobj = cJSON_GetObjectItem(cparms, "R_Matrix_data");
        if (!tempobj) { cJSON_Delete(cparms); return false; }
        for (int i = 0; i < 9; ++i) R_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

        tempobj = cJSON_GetObjectItem(cparms, "T_Vec_data");
        if (!tempobj) { cJSON_Delete(cparms); return false; }
        for (int i = 0; i < 3; ++i) T_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

        tempobj = cJSON_GetObjectItem(cparms, "Camera_Matrix_data");
        if (!tempobj) { cJSON_Delete(cparms); return false; }
        for (int i = 0; i < 9; ++i) RGB_CM_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

        tempobj = cJSON_GetObjectItem(cparms, "Distortion_Parm_data");
        if (!tempobj) { cJSON_Delete(cparms); return false; }
        for (int i = 0; i < 5; ++i) D_VEC_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

        filter_->setCameraParm(R_data, T_data, RGB_CM_data, D_VEC_data);
        cJSON_Delete(cparms);
        return true;
    }

    void publishImages(const stackframe_old_t& oldframe, const std_msgs::Header& header) {
        cv::Mat mRGB(600, 800, CV_8UC4, const_cast<uint8_t*>(oldframe.rgb));
        cv::Mat md(240, 320, CV_16UC1, const_cast<uint16_t*>(&oldframe.depth[0][0]));
        cv::Mat mi(240, 320, CV_16UC1, const_cast<uint16_t*>(&oldframe.ir[0][0]));
        cv::Mat ms(240, 320, CV_16UC1, const_cast<uint16_t*>(&oldframe.status[0][0]));

        cv::Mat mRGB_bgr;
        cv::cvtColor(mRGB, mRGB_bgr, cv::COLOR_RGBA2BGR);

        sensor_msgs::Image rgbmsg = *cv_bridge::CvImage(header, "bgr8", mRGB_bgr).toImageMsg();
        sensor_msgs::Image dmsg = *cv_bridge::CvImage(header, "mono16", md).toImageMsg();
        sensor_msgs::Image imsg = *cv_bridge::CvImage(header, "mono16", mi).toImageMsg();
        sensor_msgs::Image smsg = *cv_bridge::CvImage(header, "mono16", ms).toImageMsg();

        pub_rgb_.publish(rgbmsg);
        pub_depth_.publish(dmsg);
        pub_intensity_.publish(imsg);
        pub_status_.publish(smsg);
    }

    void publishPointCloud(const std::vector<uint16_t>& tempcali,
                           const std::vector<uint16_t>& tempcali_ir,
                           const std::vector<uint32_t>& colormap_temp,
                           const std_msgs::Header& header) {
        if (tempcali.size() < 320 * 240 || tempcali_ir.size() < 320 * 240 || colormap_temp.size() < 320 * 240) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[" << name_ << "] invalid point cloud input sizes");
            return;
        }
        if (uvf_parms_.size() < 4) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[" << name_ << "] invalid uvf parms");
            return;
        }

        sensor_msgs::PointCloud2 pcmsg;
        pcmsg.header = header;
        pcmsg.height = 240;
        pcmsg.width = 320;
        pcmsg.is_bigendian = false;
        pcmsg.point_step = 20;
        pcmsg.row_step = pcmsg.point_step * pcmsg.width;
        pcmsg.is_dense = false;
        pcmsg.fields.resize(5);

        pcmsg.fields[0].name = "x";
        pcmsg.fields[0].offset = 0;
        pcmsg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pcmsg.fields[0].count = 1;

        pcmsg.fields[1].name = "y";
        pcmsg.fields[1].offset = 4;
        pcmsg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pcmsg.fields[1].count = 1;

        pcmsg.fields[2].name = "z";
        pcmsg.fields[2].offset = 8;
        pcmsg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pcmsg.fields[2].count = 1;

        pcmsg.fields[3].name = "rgb";
        pcmsg.fields[3].offset = 12;
        pcmsg.fields[3].datatype = sensor_msgs::PointField::UINT32;
        pcmsg.fields[3].count = 1;

        pcmsg.fields[4].name = "intensity";
        pcmsg.fields[4].offset = 16;
        pcmsg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
        pcmsg.fields[4].count = 1;

        const float fox = uvf_parms_[0];
        const float foy = uvf_parms_[1];
        const float u0 = uvf_parms_[2];
        const float v0 = uvf_parms_[3];

        pcmsg.data.resize(320 * 240 * pcmsg.point_step, 0x00);
        uint8_t* ptr = pcmsg.data.data();

        // 测试点云：只保留当前数值坐标中的第一象限点。
        // 注意：该筛选只观察当前发布数据的 x/y 语义，不对 xyz 做任何修正。
        sensor_msgs::PointCloud2 first_quadrant_msg;
        first_quadrant_msg.header = header;
        first_quadrant_msg.height = 1;
        first_quadrant_msg.width = 0;
        first_quadrant_msg.is_bigendian = pcmsg.is_bigendian;
        first_quadrant_msg.point_step = pcmsg.point_step;
        first_quadrant_msg.row_step = 0;
        first_quadrant_msg.is_dense = false;
        first_quadrant_msg.fields = pcmsg.fields;
        first_quadrant_msg.data.reserve(pcmsg.data.size() / 4);

        for (int j = 0; j < 240; ++j) {
            for (int i = 0; i < 320; ++i) {
                const float cx = (static_cast<float>(i) - u0) / fox;
                const float cy = (static_cast<float>(j) - v0) / foy;
                const float dst = static_cast<float>(tempcali[j * 320 + i]) / 4.0f / 1000.0f;
                const float x = dst * cx;
                const float y = dst * cy;
                const float z = dst;

                std::memcpy(ptr + 0, &x, sizeof(float));
                std::memcpy(ptr + 4, &y, sizeof(float));
                std::memcpy(ptr + 8, &z, sizeof(float));

                const uint32_t color = colormap_temp[j * 320 + i];
                const uint32_t color_r = color & 0xff;
                const uint32_t color_g = (color & 0xff00) >> 8;
                const uint32_t color_b = (color & 0xff0000) >> 16;
                const uint32_t rgb = (color_r << 16) | (color_g << 8) | color_b;
                std::memcpy(ptr + 12, &rgb, sizeof(uint32_t));

                const float intensity = static_cast<float>(tempcali_ir[j * 320 + i]);
                std::memcpy(ptr + 16, &intensity, sizeof(float));

                // 只显示当前点云数值坐标中的第一象限：x > 0, y > 0。
                // z > 0 用于排除无效/零深度点。保留 rgb 与 intensity 字段。
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
                    x > 0.0f && y > 0.0f && z > 0.0f) {
                    first_quadrant_msg.data.insert(
                        first_quadrant_msg.data.end(),
                        ptr,
                        ptr + pcmsg.point_step);
                    ++first_quadrant_msg.width;
                }

                ptr += pcmsg.point_step;
            }
        }

        pub_pc_.publish(pcmsg);

        first_quadrant_msg.row_step = first_quadrant_msg.point_step * first_quadrant_msg.width;
        pub_first_quadrant_cloud_.publish(first_quadrant_msg);

        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "[" << name_ << "] first_quadrant_cloud points: "
            << first_quadrant_msg.width
            << " (condition: x > 0, y > 0, z > 0)");
    }

private:
    std::string name_;
    std::string host_;
    int port_;
    std::string frame_id_;

    ros::NodeHandle nh_;
    ros::Publisher pub_pc_;
    ros::Publisher pub_rgb_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_intensity_;
    ros::Publisher pub_status_;
    ros::Publisher pub_first_quadrant_cloud_;

    event_base* base_ = nullptr;
    evhttp_connection* conn_ = nullptr;
    std::vector<uint8_t> response_;
    bool request_ok_ = false;

    std::unique_ptr<Tof_Filter> filter_;
    std::vector<float> uvf_parms_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sipeed_tof_dual");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cam0_host, cam1_host;
    std::string cam0_ns, cam1_ns;
    std::string cam0_frame, cam1_frame;
    int cam0_port, cam1_port;
    double rate_hz;

    pnh.param<std::string>("cam0_host", cam0_host, "192.168.233.1");
    pnh.param<std::string>("cam1_host", cam1_host, "192.168.234.1");
    pnh.param<int>("cam0_port", cam0_port, 80);
    pnh.param<int>("cam1_port", cam1_port, 80);
    pnh.param<std::string>("cam0_ns", cam0_ns, "cam0");
    pnh.param<std::string>("cam1_ns", cam1_ns, "cam1");
    pnh.param<std::string>("cam0_frame_id", cam0_frame, "cam0_tof");
    pnh.param<std::string>("cam1_frame_id", cam1_frame, "cam1_tof");
    pnh.param<double>("rate", rate_hz, 15.0);

    MaixSenseCamera cam0(cam0_ns, cam0_host, cam0_port, cam0_frame, nh);
    MaixSenseCamera cam1(cam1_ns, cam1_host, cam1_port, cam1_frame, nh);

    bool cam0_ok = cam0.init();
    bool cam1_ok = cam1.init();

    if (!cam0_ok && !cam1_ok) {
        ROS_FATAL("Both cameras failed to initialize.");
        return 1;
    }
    if (!cam0_ok) {
        ROS_WARN("cam0 failed to initialize; only cam1 will be published.");
    }
    if (!cam1_ok) {
        ROS_WARN("cam1 failed to initialize; only cam0 will be published.");
    }

    ros::Rate loop_rate(rate_hz);
    while (ros::ok()) {
        if (cam0_ok) cam0.pollAndPublish();
        if (cam1_ok) cam1.pollAndPublish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



// rosrun sipeed_tof_cpp publisher \
//   _cam0_host:=192.168.233.1 \
//   _cam1_host:=192.168.234.1 \
//   _cam0_ns:=cam0 \
//   _cam1_ns:=cam1 \
//   _cam0_frame_id:=cam0_tof \
//   _cam1_frame_id:=cam1_tof \
//   _rate:=10.0