#include <ros/ros.h>
#include <uav_camera_stream/overlay.hpp>
#include <memory>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_srvs/SetBool.h>
#include <gst/gst.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
double monotonic() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
struct Activity {
  std::atomic<uint64_t> buffers{0}, bytes{0};
  std::atomic<double> last{0};
  void record(GstBuffer* b) {
    buffers.fetch_add(1, std::memory_order_relaxed);
    bytes.fetch_add(gst_buffer_get_size(b), std::memory_order_relaxed);
    last.store(monotonic(), std::memory_order_relaxed);
  }
};
GstPadProbeReturn countBuffer(GstPad*, GstPadProbeInfo* info, gpointer data) {
  auto* a = static_cast<Activity*>(data);
  if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) a->record(GST_PAD_PROBE_INFO_BUFFER(info));
  if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER_LIST) {
    auto* list = GST_PAD_PROBE_INFO_BUFFER_LIST(info);
    for (guint i = 0; i < gst_buffer_list_length(list); ++i) a->record(gst_buffer_list_get(list, i));
  }
  return GST_PAD_PROBE_OK;
}
struct Config {
  std::string device, destination_ip, overlay_timezone, time_format;
  int destination_port, capture_width, capture_height, capture_fps, output_fps;
  int bitrate_bps, idr_interval_frames, rtp_mtu;
  double no_frame_timeout_sec, restart_delay_sec;
  bool overlay_time, enabled;
  explicit Config(ros::NodeHandle& p) {
#define PARAM(name, value) p.param(#name, name, value)
    PARAM(device, std::string("/dev/v4l/by-id/usb-AX_USB2.0_AX_USB2.0-video-index0"));
    PARAM(destination_ip, std::string("")); PARAM(destination_port, 5600);
    PARAM(capture_width, 1280); PARAM(capture_height, 720); PARAM(capture_fps, 31);
    PARAM(output_fps, 30); PARAM(bitrate_bps, 2000000); PARAM(idr_interval_frames, 30);
    PARAM(rtp_mtu, 1200); PARAM(overlay_time, true);
    PARAM(overlay_timezone, std::string("Asia/Shanghai"));
    PARAM(time_format, std::string("%y%m%d-%H%M%S"));
    PARAM(no_frame_timeout_sec, 5.0); PARAM(restart_delay_sec, 3.0); PARAM(enabled, true);
#undef PARAM
    auto require = [](bool ok, const char* msg) { if (!ok) throw std::runtime_error(msg); };
    in_addr address{};
    require(inet_pton(AF_INET, destination_ip.c_str(), &address) == 1 &&
            (ntohl(address.s_addr) >> 24) > 0 && (ntohl(address.s_addr) >> 24) < 224 &&
            (ntohl(address.s_addr) & 255) != 255,
            "destination_ip is required: use a unicast IPv4 address, not 0.0.0.0/multicast/broadcast; use 127.0.0.1 for loopback testing");
    require(destination_port >= 1 && destination_port <= 65535, "destination_port range: 1..65535");
    require(!device.empty() && device[0] == '/', "device must be an absolute device path");
    require(capture_width >= 16 && capture_width <= 8192 && capture_height >= 16 && capture_height <= 8192 &&
            capture_width % 2 == 0 && capture_height % 2 == 0, "capture width/height must be even integers in 16..8192");
    require(capture_fps >= 1 && capture_fps <= 240 && output_fps >= 1 && output_fps <= capture_fps,
            "fps range: 1..240; output_fps must not exceed capture_fps");
    require(bitrate_bps >= 10000 && bitrate_bps <= 200000000, "bitrate_bps range: 10000..200000000 bit/s");
    require(idr_interval_frames >= 1 && idr_interval_frames <= 10000, "idr_interval_frames range: 1..10000");
    require(rtp_mtu >= 256 && rtp_mtu <= 1500, "rtp_mtu range: 256..1500 bytes");
    require(std::isfinite(no_frame_timeout_sec) && no_frame_timeout_sec >= 1 && no_frame_timeout_sec <= 300,
            "no_frame_timeout_sec range: 1..300");
    require(std::isfinite(restart_delay_sec) && restart_delay_sec >= 1 && restart_delay_sec <= 300,
            "restart_delay_sec range: 1..300");
    require(!overlay_timezone.empty() && overlay_timezone[0] != '/' && overlay_timezone.find("..") == std::string::npos,
            "overlay_timezone must be a valid IANA zoneinfo name");
    std::ifstream zone("/usr/share/zoneinfo/" + overlay_timezone, std::ios::binary);
    char magic[4]{}; zone.read(magic, 4);
    require(zone && std::string(magic, 4) == "TZif", "overlay_timezone is missing from /usr/share/zoneinfo or is not a TZif file");
    require(!time_format.empty() && time_format.size() <= 128 && time_format.find("%z") == std::string::npos &&
            time_format.find("%Z") == std::string::npos && time_format.find("%n") == std::string::npos &&
            time_format.find("%t") == std::string::npos && time_format.find('\n') == std::string::npos && time_format.find('\r') == std::string::npos,
            "time_format must be one line, 1..128 bytes; compact timezone is appended automatically (omit %z/%Z)");
    require(setenv("TZ", overlay_timezone.c_str(), 1) == 0, "Failed to set process TZ");
    tzset();  // Before gst_init and all streaming threads; never changes system timezone.
    ROS_INFO_STREAM("Process overlay timezone " << overlay_timezone << ": " << uav_camera_stream::wallTime(time(nullptr), time_format));
  }
};
// Query discrete MJPEG modes without changing device settings. No guessed fallback.
void verifyMode(const Config& c) {
  int fd = open(c.device.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) throw std::runtime_error("Cannot open camera " + c.device + ": " + std::string(strerror(errno)));
  bool supported = false;
  v4l2_frmsizeenum size{}; size.pixel_format = V4L2_PIX_FMT_MJPEG;
  for (size.index = 0; ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
    if (size.type != V4L2_FRMSIZE_TYPE_DISCRETE || size.discrete.width != static_cast<unsigned>(c.capture_width) ||
        size.discrete.height != static_cast<unsigned>(c.capture_height)) continue;
    v4l2_frmivalenum rate{}; rate.pixel_format = V4L2_PIX_FMT_MJPEG;
    rate.width = c.capture_width; rate.height = c.capture_height;
    for (rate.index = 0; ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &rate) == 0; ++rate.index)
      if (rate.type == V4L2_FRMIVAL_TYPE_DISCRETE &&
          static_cast<uint64_t>(rate.discrete.numerator) * c.capture_fps == rate.discrete.denominator) supported = true;
  }
  close(fd);
  if (!supported) throw std::runtime_error("Requested discrete MJPEG width/height/fps is not enumerated; check v4l2-ctl --list-formats-ext; no mode fallback");
}
std::string quote(const std::string& s) {
  gchar* escaped = g_strescape(s.c_str(), nullptr);
  std::string result = "\"" + std::string(escaped) + "\""; g_free(escaped); return result;
}

class CameraStream {
  ros::NodeHandle p_{"~"};
  Config c_;
  std::unique_ptr<uav_camera_stream::Overlay> overlay_;
  GstElement* hud_{nullptr};
  double overlay_fps_{0}, overlay_mbps_{0}, last_overlay_update_{0};
  bool overlay_rates_ready_{false};
  std::string overlay_text_;
  void updateOverlay(double now, bool force=false) {
    if (!hud_ || (!force && now-last_overlay_update_ < 1.0/overlay_->layout().update_hz)) return;
    const auto text = uav_camera_stream::render(overlay_->layout(), overlay_->telemetry(), time(nullptr), now,
                                              c_.time_format, c_.overlay_time, overlay_rates_ready_, overlay_fps_, overlay_mbps_);
    if (text != overlay_text_) {
      gchar* escaped = g_markup_escape_text(text.c_str(), -1);
      g_object_set(hud_, "text", escaped, nullptr); g_free(escaped); overlay_text_=text;
    }
    last_overlay_update_=now;
  }
  ros::Publisher diagnostics_;
  ros::ServiceServer enable_service_;
  GstElement* pipeline_{nullptr}; GstBus* bus_{nullptr};
  struct Probe { GstPad* pad; gulong id; };
  std::vector<Probe> probes_;
  Activity input_, raw_, encoded_, rtp_;
  bool enabled_, attempted_{false};
  unsigned restarts_{0};
  double started_{0}, next_attempt_{0}, last_report_{monotonic()};
  uint64_t prev_input_{0}, prev_raw_{0}, prev_encoded_{0}, prev_rtp_{0}, prev_udp_{0}, udp_completed_{0};
  std::string error_, capture_caps_, encoder_caps_;
  void probe(const char* element, const char* pad_name, Activity& activity) {
    GstElement* e = gst_bin_get_by_name(GST_BIN(pipeline_), element);
    GstPad* pad = e ? gst_element_get_static_pad(e, pad_name) : nullptr;
    if (e) gst_object_unref(e);
    if (!pad) throw std::runtime_error(std::string("Missing probe pad: ") + element);
    gulong id = gst_pad_add_probe(pad, static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST),
                                  countBuffer, &activity, nullptr);
    probes_.push_back({pad, id});
  }
  uint64_t socketBytes() const {
    guint64 bytes = 0;
    if (pipeline_) {
      GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_), "udp");
      if (sink) { g_object_get(sink, "bytes-served", &bytes, nullptr); gst_object_unref(sink); }
    }
    return udp_completed_ + bytes;
  }
  void stop() {
    if (!pipeline_) return;
    // NULL joins GStreamer tasks before releasing pad probes or their counters.
    const uint64_t submitted = socketBytes();
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    for (const auto& p : probes_) { gst_pad_remove_probe(p.pad, p.id); gst_object_unref(p.pad); }
    probes_.clear();
    if (hud_) gst_object_unref(hud_);
    hud_=nullptr; overlay_rates_ready_=false;
    if (bus_) gst_object_unref(bus_);
    bus_ = nullptr;
    gst_object_unref(pipeline_); pipeline_ = nullptr;
    udp_completed_ = submitted;
    ROS_INFO("Pipeline stopped; camera released");
  }
  void fail(const std::string& message) {
    error_ = message; ROS_ERROR_STREAM(message);
    stop(); next_attempt_ = monotonic() + c_.restart_delay_sec;
  }
  void start() {
    if (attempted_) ++restarts_;
    attempted_ = true;
    try {
      verifyMode(c_);
      std::ostringstream s;
      s << "v4l2src name=camera device=" << quote(c_.device) << " do-timestamp=true ! image/jpeg,width="
        << c_.capture_width << ",height=" << c_.capture_height << ",framerate=" << c_.capture_fps << "/1"
        << " ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream"
        << " ! jpegdec name=decoder ! videorate drop-only=true ! video/x-raw,framerate=" << c_.output_fps << "/1"
        << " ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream"
        << " ! videoconvert ! video/x-raw,format=I420";
      s << " ! textoverlay name=hud shaded-background=false draw-outline=true draw-shadow=false"
        << " outline-color=4278190080 color=4294967295 halignment=left valignment=top line-alignment=left"
        << " auto-resize=false wrap-mode=none wait-text=false";
      // Fixed system-memory caps prevent nvvidconv's NULL caps copy on overlay meta negotiation (R35.6.5).
      s << " ! video/x-raw,format=I420,width=" << c_.capture_width << ",height=" << c_.capture_height
        << ",framerate=" << c_.output_fps << "/1 ! nvvidconv"
        << " ! video/x-raw(memory:NVMM),format=NV12"
        << " ! nvv4l2h264enc name=encoder bitrate=" << c_.bitrate_bps << " control-rate=1 iframeinterval="
        << c_.idr_interval_frames << " idrinterval=" << c_.idr_interval_frames << " insert-sps-pps=true"
        << " ! video/x-h264,stream-format=byte-stream,alignment=au"
        << " ! rtph264pay name=pay pt=96 config-interval=-1 mtu=" << c_.rtp_mtu
        << " ! udpsink name=udp host=" << quote(c_.destination_ip) << " port=" << c_.destination_port
        << " sync=false async=false";
      GError* e = nullptr;
      pipeline_ = gst_parse_launch(s.str().c_str(), &e);
      if (e) { std::string message = e->message; g_error_free(e); throw std::runtime_error(message); }
      if (!pipeline_) throw std::runtime_error("gst_parse_launch did not create a pipeline");
      bus_ = gst_element_get_bus(pipeline_);
      hud_=gst_bin_get_by_name(GST_BIN(pipeline_), "hud");
      if (!hud_) throw std::runtime_error("Missing textoverlay element");
      const auto& layout=overlay_->layout();
      g_object_set(hud_, "font-desc", layout.font_desc.c_str(), "xpad", layout.xpad, "ypad", layout.ypad, nullptr);
      overlay_text_.clear(); overlay_rates_ready_=false; last_overlay_update_=0;
      updateOverlay(monotonic(), true);
      probe("decoder", "src", input_);  // Decoded MJPEG: exactly one raw frame per buffer.
      probe("encoder", "sink", raw_);  // NV12 raw frames admitted to encoder, not encoded buffer counts.
      probe("encoder", "src", encoded_);
      probe("pay", "src", rtp_);  // Includes GstBufferList packets.
      started_ = monotonic(); capture_caps_.clear(); encoder_caps_.clear();
      if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        throw std::runtime_error("Failed to enter PLAYING");
      ROS_INFO_STREAM("Starting RTP/UDP -> " << c_.destination_ip << ':' << c_.destination_port);
    } catch (const std::exception& e) { fail(e.what()); }
  }
  std::string caps(const char* element, const char* name) const {
    GstElement* e = gst_bin_get_by_name(GST_BIN(pipeline_), element);
    GstPad* p = gst_element_get_static_pad(e, name);
    GstCaps* c = gst_pad_get_current_caps(p);
    std::string result;
    if (c) { gchar* text = gst_caps_to_string(c); result = text; g_free(text); gst_caps_unref(c); }
    gst_object_unref(p); gst_object_unref(e); return result;
  }
  void busAndWatchdog() {
    while (pipeline_) {
      GstMessage* m = gst_bus_pop(bus_);
      if (!m) break;
      std::string failure;
      if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_ERROR || GST_MESSAGE_TYPE(m) == GST_MESSAGE_WARNING) {
        GError* e = nullptr; gchar* detail = nullptr;
        const bool fatal = GST_MESSAGE_TYPE(m) == GST_MESSAGE_ERROR;
        if (fatal) gst_message_parse_error(m, &e, &detail); else gst_message_parse_warning(m, &e, &detail);
        std::string msg = std::string(GST_OBJECT_NAME(m->src)) + ": " + (e ? e->message : "unknown") +
                          " [" + (detail ? detail : "") + "]";
        if (fatal) failure = msg; else ROS_WARN_STREAM(msg);
        if (e) g_error_free(e);
        g_free(detail);
      } else if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_EOS) failure = "Unexpected EOS";
      gst_message_unref(m);
      if (!failure.empty()) { fail(failure); return; }
    }
    if (!pipeline_) return;
    const double now = monotonic();
    if (now - std::max(started_, input_.last.load()) > c_.no_frame_timeout_sec)
      fail("watchdog: decoded input frames stopped");
    else if (now - std::max(started_, encoded_.last.load()) > c_.no_frame_timeout_sec)
      fail("watchdog: encoded output stopped");
  }
  bool setEnabled(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (enabled_ != req.data) {
      enabled_ = req.data;
      if (!enabled_) stop(); else next_attempt_ = monotonic();
    }
    res.success = true;
    res.message = enabled_ ? "Enabled; state machine will start/retry. Check diagnostics" : "Disabled; pipeline stopped and camera released";
    return true;
  }
  void report(double now) {
    const double dt = now - last_report_;
    diagnostic_msgs::DiagnosticArray array; array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus d; d.name = ros::this_node::getName() + "/stream"; d.hardware_id = c_.device;
    const bool active = pipeline_ && input_.last.load() >= started_ && encoded_.last.load() >= started_;
    d.level = !enabled_ ? d.OK : !pipeline_ ? d.ERROR : active ? d.OK : d.WARN;
    d.message = !enabled_ ? "disabled" : !pipeline_ ? "retry_wait" : active ? "streaming_local_only" : "starting";
    auto kv = [&d](const std::string& k, const std::string& v) { diagnostic_msgs::KeyValue p; p.key = k; p.value = v; d.values.push_back(p); };
    auto number = [&kv](const std::string& k, double v) { std::ostringstream s; s.precision(8); s << v; kv(k, s.str()); };
    auto age = [now](const Activity& a) { double t = a.last.load(); return t > 0 ? now - t : -1.0; };
    const uint64_t in = input_.buffers.load(), raw = raw_.buffers.load(), enc = encoded_.bytes.load();
    const uint64_t rtp = rtp_.bytes.load(), udp = socketBytes();
    number("window_sec", dt); number("input_decoded_fps", (in-prev_input_)/dt);
    number("encoder_input_raw_fps", (raw-prev_raw_)/dt);
    number("encoded_bytes_per_sec", (enc-prev_encoded_)/dt);
    number("rtp_offered_bytes_per_sec", (rtp-prev_rtp_)/dt);
    number("udp_socket_submitted_bytes_per_sec", (udp-prev_udp_)/dt);
    number("last_input_age_sec", age(input_)); number("last_encoded_age_sec", age(encoded_)); number("last_rtp_age_sec", age(rtp_));
    number("input_frames_total", in); number("encoder_input_frames_total", raw);
    number("encoded_buffers_total_not_frames", encoded_.buffers.load()); number("rtp_packets_offered_total", rtp_.buffers.load());
    number("udp_socket_submitted_bytes_total", udp); number("restart_attempts", restarts_);
    kv("last_error", error_); kv("destination", c_.destination_ip + ":" + std::to_string(c_.destination_port));
    kv("overlay_timezone", c_.overlay_timezone); kv("ground_reception", "unknown; UDP has no receiver acknowledgement");
    if (pipeline_) {
      auto capture = caps("camera", "src"), encoder = caps("encoder", "src");
      if (!capture.empty() && capture != capture_caps_) { capture_caps_ = capture; ROS_INFO_STREAM("Negotiated capture caps: " << capture); }
      if (!encoder.empty() && encoder != encoder_caps_) { encoder_caps_ = encoder; ROS_INFO_STREAM("Negotiated encoded caps: " << encoder); }
      kv("encoder_input_caps", caps("encoder", "sink"));
    }
    kv("capture_caps", capture_caps_); kv("encoded_caps", encoder_caps_);
    overlay_fps_=(raw-prev_raw_)/dt;
    overlay_mbps_=(udp-prev_udp_)*8.0/dt/1000000.0;
    overlay_rates_ready_=active;
    updateOverlay(now, true);
    kv("overlay_text", overlay_text_);
    number("overlay_fps", overlay_fps_); number("overlay_udp_mbps", overlay_mbps_);
    for (const auto& field : overlay_->layout().fields) if (field.enabled) {
      const std::string prefix=std::string("telemetry_")+uav_camera_stream::metricName(field.id);
      const auto state=overlay_->telemetry().state(field.id,now,overlay_->layout().telemetry_timeout_sec);
      kv(prefix+"_state",state);
      if (state=="ok") number(prefix+"_value",overlay_->telemetry().get(field.id).value);
    }
    array.status.push_back(d); diagnostics_.publish(array);
    ROS_INFO_STREAM("state=" << d.message << " input=" << (in-prev_input_)/dt << " fps encoder_input=" << (raw-prev_raw_)/dt
                    << " fps encoded=" << (enc-prev_encoded_)*8/dt << " bit/s UDP socket=" << (udp-prev_udp_)/dt << " B/s");
    prev_input_=in; prev_raw_=raw; prev_encoded_=enc; prev_rtp_=rtp; prev_udp_=udp; last_report_=now;
  }
 public:
  CameraStream() : c_(p_), enabled_(c_.enabled) {
    overlay_.reset(new uav_camera_stream::Overlay(p_));
    gst_init(nullptr, nullptr);
    diagnostics_ = p_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    enable_service_ = p_.advertiseService("set_enabled", &CameraStream::setEnabled, this);
  }
  ~CameraStream() { stop(); }
  void run() {
    // Only this thread and spinOnce service callbacks mutate pipeline ownership.
    while (ros::ok()) {
      ros::spinOnce();
      if (enabled_ && !pipeline_ && monotonic() >= next_attempt_) start();
      busAndWatchdog();
      double now = monotonic(); if (now - last_report_ >= 1.0) report(now);
      updateOverlay(now);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
};
}  // namespace
int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_stream");
  try { CameraStream node; node.run(); }
  catch (const std::exception& e) {
    // A failed constructor may have destroyed the last NodeHandle and stopped rosconsole.
    std::cerr << "camera_stream startup error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
