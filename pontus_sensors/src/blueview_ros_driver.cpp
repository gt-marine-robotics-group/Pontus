#include <blueview_ros_driver.hpp>

BlueViewRosDriver::BlueViewRosDriver() : Node("blueview_driver"), node_handle_(std::shared_ptr<BlueViewRosDriver>(this, [](auto *) {})),
image_transport(node_handle_) {
    initParams();
}

void BlueViewRosDriver::initParams() {
    // Get sonar frame_id for header
    this->declare_parameter("sonar_frame", "sonar_0");

    // Determine which topics to publish
    this->declare_parameter("grayscale/enable", true);
    this->declare_parameter("color/enable", false);
    this->declare_parameter("raw/enable", false);

    this->declare_parameter("color/map_file", "");
    this->declare_parameter("file", "");
    this->declare_parameter("device", "");

    this->declare_parameter("range/start", 0.0);
    this->declare_parameter("range/stop", 5.0);

    this->declare_parameter("fluid_type", "freshwater");
    this->declare_parameter("sound_speed", 1440);
    this->declare_parameter("dynamic_power_management", false);
    this->declare_parameter("range_profile_intensity_threshold", 2000);
    this->declare_parameter("noise_threshold", 0.2);
    this->declare_parameter("period_seconds", -1.0);

    this->get_parameter("sonar_frame", frame_id);
    this->get_parameter("grayscale/enable", do_grayscale);
    this->get_parameter("color/enable", do_color);
    this->get_parameter("raw/enable",do_raw);

    if (do_grayscale) {
        grayscale_img.reset(new cv_bridge::CvImage());
        grayscale_img->encoding = "mono16";
        grayscale_img->header.frame_id = frame_id;
        grayscale_pub = image_transport.advertise("image_mono", 1);
    }

    if (do_color) {
        // Get and load color map
        std::string color_map_file;
        if (this->get_parameter("color/map_file", color_map_file))
            sonar.loadColorMapper(color_map_file);

        color_img.reset(new cv_bridge::CvImage());
        color_img->encoding = "bgra8";
        color_img->header.frame_id = frame_id;
        color_pub = image_transport.advertise("image_color", 1);
    }

    if (do_raw) {
        ping_msg.reset(new pontus_msgs::msg::Blueviewping());
        ping_msg->header.frame_id = frame_id;
        raw_pub = this->create_publisher<pontus_msgs::msg::Blueviewping>("ranges",10);
    }

    std::string params;
    if (this->get_parameter("file", params))
        sonar.init(BlueViewSonar::ConnectionType::FILE, params);
    else if (this->get_parameter("device", params))
        sonar.init(BlueViewSonar::ConnectionType::DEVICE, params);
    else
        throw std::runtime_error("Can not connect: neither 'file' or 'device' param set");

    // Set Ranges, meters
    BVTSDK::Head &head = sonar.getHead();
    float range_lower, range_upper;
    if (this->get_parameter("range/start", range_lower)) {
        head.SetStartRange(range_lower);
    }
    if (this->get_parameter("range/stop", range_upper)) {
        head.SetStopRange(range_upper);
    }

    // Handle fluid type, enum string
    std::string fluid_type;
    if (this->get_parameter("fluid_type", fluid_type)) {
        if (fluid_type == "saltwater")
            head.SetFluidType(BVTSDK::FluidType::Saltwater);
        else if (fluid_type == "freshwater")
            head.SetFluidType(BVTSDK::FluidType::Freshwater);
        else if (fluid_type == "other")
            head.SetFluidType(BVTSDK::FluidType::Other);
        else
            RCLCPP_ERROR(this->get_logger(),"Fluid type (%s) invalid, must be 'saltwater', 'freshwater', "
                                "or 'other'",
                                fluid_type.c_str());
    }

    // Set sound speed, m/s
    int sound_speed;
    if (this->get_parameter("sound_speed", sound_speed))
        head.SetSoundSpeed(sound_speed);

    float range_resolution;
    if (this->get_parameter("range_resolution", range_resolution))
        head.SetRangeResolution(range_resolution);

    // Set analog gain adjustment in dB
    float gain;
    if (this->get_parameter("gain_adjustment", gain))
        head.SetGainAdjustment(gain);

    // Set "time variable analog gain", in dB/meter,
    float tvg;
    if (this->get_parameter("tvg_slope", tvg))
        head.SetTVGSlope(tvg);

    // Set dynamic power management
    bool dynamic_power;
    if (this->get_parameter("dynamic_power_management", dynamic_power))
        head.SetDynamicPowerManagement(dynamic_power);

    // Set ping interval
    float ping_interval;
    if (this->get_parameter("ping_interval", ping_interval))
        head.SetPingInterval(ping_interval);
    sonar.updateHead();

    int range_profile_thresh;
    if (this->get_parameter("range_profile_intensity_threshold", range_profile_thresh))
        sonar.SetRangeProfileMinIntensity(range_profile_thresh);

    float noise_threshold;
    if (this->get_parameter("noise_threshold", noise_threshold))
        sonar.SetRangeProfileMinIntensity(noise_threshold);

    // Start loop
    this->get_parameter("period_seconds", period_seconds_);
}

void BlueViewRosDriver::run() {
    if (period_seconds_ <= 0.0) {
        while (rclcpp::ok()) {
            get_ping();
            rclcpp::spin_some(shared_from_this());
        }
    } else {
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period_seconds_),
                                                     std::bind(&BlueViewRosDriver::loop, this));
        rclcpp::spin(shared_from_this());
    }
}

void BlueViewRosDriver::get_ping() {
    if (!sonar.getNextPing()) {
        RCLCPP_WARN(this->get_logger(),"No pings remaining in file, shutting down...");
        rclcpp::shutdown();
        return;
    }

    // Do images
    if (do_grayscale || do_color) {
        sonar.generateImage();
        if (do_grayscale) {
            sonar.getGrayscaleImage(grayscale_img->image);
            grayscale_pub.publish(grayscale_img->toImageMsg());
        }

        if (do_color) {
            sonar.getColorImage(color_img->image);
            color_pub.publish(color_img->toImageMsg());
        }

        if (do_raw) {
            ping_msg->header.stamp = rclcpp::Node::now();
            sonar.getRanges(ping_msg->bearings, ping_msg->ranges, ping_msg->intensities);
            raw_pub->publish(*ping_msg);
        }
    }
}

void BlueViewRosDriver::loop() {
    get_ping();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto driver = std::make_shared<BlueViewRosDriver>();

    try {
        driver->run();
    } catch (const BVTSDK::SdkException &err) {
        RCLCPP_FATAL(driver->get_logger(),"Exception thrown in Blue View SDK (Error #%d):\n\t%s: %s\n", err.ReturnCode(), err.ErrorName().c_str(),
                            err.ErrorMessage().c_str());
    } catch (const std::runtime_error &err) {
        RCLCPP_FATAL(driver->get_logger(),"Exception: %s", err.what());
    }
}
