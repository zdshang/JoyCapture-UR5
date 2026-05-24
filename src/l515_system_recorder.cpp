// Dedicated RealSense recorder service for systems that prefer a native L515 path.
//
// It implements the same newline-delimited JSON command protocol as
// d455_recorder_service.py, but keeps capture in C++ for lower overhead on
// Linux lab machines.
#include <librealsense2/rs.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

struct FrameRow {
    // One row in the camera timestamp CSV used for robot-camera alignment.
    int frame_idx = 0;
    long long host_monotonic_ns = 0;
    long long system_time_ns = 0;
    unsigned long long rs_frame_number = 0;
    double rs_timestamp_ms = 0.0;
    double t_rel_s = 0.0;
    int pixel_count = 0;
};

struct RecorderState {
    // Shared service state. Socket handlers and the capture thread both touch
    // these fields, so updates to frame_rows and paths are guarded by mu.
    std::mutex mu;
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;
    std::unique_ptr<rs2::config> config;
    std::atomic<bool> running{true};
    std::atomic<bool> recording{false};
    std::atomic<bool> stop_capture{false};
    std::thread capture_thread;
    std::string camera_label = "camera";
    std::string device_name = "";
    std::string serial = "";
    std::string pipeline_mode = "stopped";
    std::string video_path = "";
    std::string bag_path = "";
    std::string frame_ts_path = "";
    std::string metadata_path = "";
    std::string intrinsics_path = "";
    std::string frame_export_dir = "";
    std::string depth_csv_dir = "";
    std::string video_codec = "ffmpeg_mpeg4";
    double video_fps = 30.0;
    int video_w = 640;
    int video_h = 480;
    int export_frame_every_n = 1;
    int max_export_frames = 0;
    std::atomic<long long> record_start_host_ns{0};
    bool postprocess_required = false;
    bool depth_enabled = false;
    bool infra_enabled = false;
    bool bag_enabled = false;
    double depth_scale = 0.0;
    std::vector<FrameRow> frame_rows;
    int frame_idx = 0;
};

static long long monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

static long long system_time_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static std::string json_escape(const std::string& s) {
    std::ostringstream out;
    for (char c : s) {
        switch (c) {
            case '\\':
                out << "\\\\";
                break;
            case '"':
                out << "\\\"";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << c;
                break;
        }
    }
    return out.str();
}

static std::string kv(const std::string& key, const std::string& value, bool quote = true) {
    std::ostringstream out;
    out << "\"" << json_escape(key) << "\":";
    if (quote) {
        out << "\"" << json_escape(value) << "\"";
    } else {
        out << value;
    }
    return out.str();
}

static std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

static std::string get_json_string(const std::string& body, const std::string& key, const std::string& fallback = "") {
    std::string needle = "\"" + key + "\"";
    size_t p = body.find(needle);
    if (p == std::string::npos) {
        return fallback;
    }
    p = body.find(':', p);
    if (p == std::string::npos) {
        return fallback;
    }
    ++p;
    while (p < body.size() && std::isspace(static_cast<unsigned char>(body[p]))) {
        ++p;
    }
    if (p >= body.size() || body[p] != '"') {
        return fallback;
    }
    ++p;
    std::ostringstream out;
    while (p < body.size()) {
        char c = body[p++];
        if (c == '\\' && p < body.size()) {
            char n = body[p++];
            switch (n) {
                case 'n':
                    out << '\n';
                    break;
                case 'r':
                    out << '\r';
                    break;
                case 't':
                    out << '\t';
                    break;
                case '\\':
                case '"':
                case '/':
                    out << n;
                    break;
                default:
                    out << n;
                    break;
            }
        } else if (c == '"') {
            break;
        } else {
            out << c;
        }
    }
    return out.str();
}

static long long get_json_int(const std::string& body, const std::string& key, long long fallback = 0) {
    std::string needle = "\"" + key + "\"";
    size_t p = body.find(needle);
    if (p == std::string::npos) {
        return fallback;
    }
    p = body.find(':', p);
    if (p == std::string::npos) {
        return fallback;
    }
    ++p;
    while (p < body.size() && std::isspace(static_cast<unsigned char>(body[p]))) {
        ++p;
    }
    size_t end = p;
    while (end < body.size() &&
           (std::isdigit(static_cast<unsigned char>(body[end])) || body[end] == '-' || body[end] == '+')) {
        ++end;
    }
    std::string raw = trim(body.substr(p, end - p));
    if (raw.empty()) {
        return fallback;
    }
    try {
        return std::stoll(raw);
    } catch (...) {
        return fallback;
    }
}

static bool get_json_bool(const std::string& body, const std::string& key, bool fallback = false) {
    std::string needle = "\"" + key + "\"";
    size_t p = body.find(needle);
    if (p == std::string::npos) {
        return fallback;
    }
    p = body.find(':', p);
    if (p == std::string::npos) {
        return fallback;
    }
    ++p;
    std::string rest = trim(body.substr(p, 8));
    if (rest.rfind("true", 0) == 0) {
        return true;
    }
    if (rest.rfind("false", 0) == 0) {
        return false;
    }
    return fallback;
}

static void write_frame_ts_csv(RecorderState& st) {
    if (st.frame_ts_path.empty()) {
        return;
    }
    fs::create_directories(fs::path(st.frame_ts_path).parent_path());
    std::ofstream f(st.frame_ts_path, std::ios::trunc);
    f << "frame_idx,host_monotonic_ns,system_time_ns,rs_frame_number,rs_timestamp_ms,t_rel_s,pixel_count\n";
    for (const auto& row : st.frame_rows) {
        f << row.frame_idx << ','
          << row.host_monotonic_ns << ','
          << row.system_time_ns << ','
          << row.rs_frame_number << ','
          << row.rs_timestamp_ms << ','
          << row.t_rel_s << ','
          << row.pixel_count << '\n';
    }
}

static void write_intrinsics_json(RecorderState& st) {
    if (st.intrinsics_path.empty()) {
        return;
    }
    fs::create_directories(fs::path(st.intrinsics_path).parent_path());
    std::ofstream f(st.intrinsics_path, std::ios::trunc);
    f << "{\n";
    f << "  " << kv("serial", st.serial) << ",\n";
    f << "  " << kv("device_name", st.device_name) << ",\n";
    f << "  " << kv("depth_scale", std::to_string(st.depth_scale), false) << ",\n";
    f << "  \"streams\": [\n";
    bool first = true;
    try {
        for (auto&& sp : st.profile.get_streams()) {
            rs2::video_stream_profile vsp = sp.as<rs2::video_stream_profile>();
            auto intr = vsp.get_intrinsics();
            if (!first) {
                f << ",\n";
            }
            first = false;
            f << "    {\n";
            f << "      " << kv("stream_type", rs2_stream_to_string(vsp.stream_type())) << ",\n";
            f << "      " << kv("stream_index", std::to_string(vsp.stream_index()), false) << ",\n";
            f << "      " << kv("width", std::to_string(vsp.width()), false) << ",\n";
            f << "      " << kv("height", std::to_string(vsp.height()), false) << ",\n";
            f << "      " << kv("fps", std::to_string(vsp.fps()), false) << ",\n";
            f << "      " << kv("fx", std::to_string(intr.fx), false) << ",\n";
            f << "      " << kv("fy", std::to_string(intr.fy), false) << ",\n";
            f << "      " << kv("ppx", std::to_string(intr.ppx), false) << ",\n";
            f << "      " << kv("ppy", std::to_string(intr.ppy), false) << "\n";
            f << "    }";
        }
    } catch (...) {
    }
    f << "\n  ]\n";
    f << "}\n";
}

static void write_metadata_json(RecorderState& st) {
    if (st.metadata_path.empty()) {
        return;
    }
    fs::create_directories(fs::path(st.metadata_path).parent_path());
    std::ofstream f(st.metadata_path, std::ios::trunc);
    f << "{\n";
    f << "  " << kv("camera_label", st.camera_label) << ",\n";
    f << "  " << kv("device_name", st.device_name) << ",\n";
    f << "  " << kv("serial", st.serial) << ",\n";
    f << "  " << kv("pipeline_mode", st.pipeline_mode) << ",\n";
    f << "  " << kv("video_path", st.video_path) << ",\n";
    f << "  " << kv("bag_path", st.bag_path) << ",\n";
    f << "  " << kv("frame_ts_path", st.frame_ts_path) << ",\n";
    f << "  " << kv("intrinsics_path", st.intrinsics_path) << ",\n";
    f << "  " << kv("frame_export_dir", st.frame_export_dir) << ",\n";
    f << "  " << kv("depth_csv_dir", st.depth_csv_dir) << ",\n";
    f << "  " << kv("video_codec", st.video_codec) << ",\n";
    f << "  " << kv("video_fps", std::to_string(st.video_fps), false) << ",\n";
    f << "  \"video_size\": [" << st.video_w << ", " << st.video_h << "],\n";
    f << "  " << kv("frame_count", std::to_string(st.frame_rows.size()), false) << ",\n";
    f << "  " << kv("postprocess_required", st.postprocess_required ? "true" : "false", false) << ",\n";
    f << "  " << kv("postprocess_done", "false", false) << ",\n";
    f << "  " << kv("depth_scale", std::to_string(st.depth_scale), false) << ",\n";
    f << "  " << kv("depth_enabled", st.depth_enabled ? "true" : "false", false) << ",\n";
    f << "  " << kv("infra_enabled", st.infra_enabled ? "true" : "false", false) << ",\n";
    f << "  " << kv("export_frame_every_n", std::to_string(st.export_frame_every_n), false) << ",\n";
    f << "  " << kv("export_max_frames", std::to_string(st.max_export_frames), false) << ",\n";
    f << "  " << kv("record_start_host_ns", std::to_string(st.record_start_host_ns.load()), false) << "\n";
    f << "}\n";
}

static std::string json_ok_ping(RecorderState& st) {
    std::ostringstream out;
    out << "{";
    out << kv("ok", "true", false) << ",";
    out << kv("recording", st.recording ? "true" : "false", false) << ",";
    out << kv("pipeline_mode", st.pipeline_mode) << ",";
    out << kv("serial", st.serial) << ",";
    out << kv("device_name", st.device_name) << ",";
    out << kv("frame_count", std::to_string(st.frame_rows.size()), false);
    out << "}\n";
    return out.str();
}

static std::string json_status(RecorderState& st) {
    write_frame_ts_csv(st);
    write_metadata_json(st);
    write_intrinsics_json(st);
    long long video_size = 0;
    if (!st.video_path.empty() && fs::exists(st.video_path)) {
        video_size = static_cast<long long>(fs::file_size(st.video_path));
    }
    std::ostringstream out;
    out << "{";
    out << kv("ok", "true", false) << ",";
    out << kv("recording", st.recording ? "true" : "false", false) << ",";
    out << kv("video_path", st.video_path) << ",";
    out << kv("video_size", std::to_string(video_size), false) << ",";
    out << kv("bag_path", st.bag_path) << ",";
    out << kv("frame_ts_path", st.frame_ts_path) << ",";
    out << kv("metadata_path", st.metadata_path) << ",";
    out << kv("intrinsics_path", st.intrinsics_path) << ",";
    out << kv("frame_export_dir", st.frame_export_dir) << ",";
    out << kv("depth_csv_dir", st.depth_csv_dir) << ",";
    out << kv("frame_count", std::to_string(st.frame_rows.size()), false) << ",";
    out << kv("device_name", st.device_name) << ",";
    out << kv("serial", st.serial) << ",";
    out << kv("pipeline_mode", st.pipeline_mode) << ",";
    out << kv("camera_label", st.camera_label) << ",";
    out << kv("camera_fps", std::to_string(static_cast<int>(st.video_fps)), false) << ",";
    out << kv("started_depth", st.depth_enabled ? "true" : "false", false) << ",";
    out << kv("started_infra", st.infra_enabled ? "true" : "false", false) << ",";
    out << kv("started_bag", st.bag_enabled ? "true" : "false", false) << ",";
    out << kv("record_start_host_ns", std::to_string(st.record_start_host_ns.load()), false) << ",";
    out << kv("video_codec", st.video_codec) << ",";
    out << kv("postprocess_required", st.postprocess_required ? "true" : "false", false) << ",";
    out << kv("export_frame_every_n", std::to_string(st.export_frame_every_n), false) << ",";
    out << kv("export_max_frames", std::to_string(st.max_export_frames), false);
    out << "}\n";
    return out.str();
}

static std::string json_error(const std::string& msg) {
    std::ostringstream out;
    out << "{";
    out << kv("ok", "false", false) << ",";
    out << kv("error", msg);
    out << "}\n";
    return out.str();
}

static bool select_l515_device(RecorderState& st, std::string preferred_name, std::string& err) {
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0) {
        err = "no realsense devices detected by system librealsense";
        return false;
    }
    std::string pref = preferred_name;
    for (auto& ch : pref) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    for (auto&& dev : list) {
        std::string name = dev.supports(RS2_CAMERA_INFO_NAME) ? dev.get_info(RS2_CAMERA_INFO_NAME) : "";
        std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
        std::string lname = name;
        for (auto& ch : lname) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        if (pref.empty() || lname.find(pref) != std::string::npos) {
            st.device_name = name;
            st.serial = serial;
            return true;
        }
    }
    std::ostringstream avail;
    bool first = true;
    for (auto&& dev : list) {
        std::string name = dev.supports(RS2_CAMERA_INFO_NAME) ? dev.get_info(RS2_CAMERA_INFO_NAME) : "";
        std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
        if (!first) {
            avail << ", ";
        }
        first = false;
        avail << name << " [" << serial << "]";
    }
    err = "preferred device '" + preferred_name + "' not found; available: " + avail.str();
    return false;
}

static void remove_partial_bag(const RecorderState& st) {
    if (st.bag_path.empty()) {
        return;
    }
    try {
        fs::path p(st.bag_path);
        if (fs::exists(p)) {
            fs::remove(p);
        }
    } catch (...) {
    }
}

static bool wait_for_first_color_frame(RecorderState& st, std::string& err, int timeout_ms = 2000, int attempts = 2) {
    err.clear();
    for (int i = 0; i < std::max(1, attempts); ++i) {
        try {
            auto frames = st.pipe.wait_for_frames(timeout_ms);
            auto color = frames.get_color_frame();
            if (!color) {
                err = "first frameset had no color frame";
                continue;
            }
            static_cast<void>(color.get_data());
            return true;
        } catch (const rs2::error& e) {
            err = e.what();
        } catch (const std::exception& e) {
            err = e.what();
        } catch (...) {
            err = "unknown wait_for_frames failure";
        }
    }
    if (err.empty()) {
        err = "no color frame received after pipeline start";
    }
    return false;
}

static void capture_loop(RecorderState* st_ptr) {
    RecorderState& st = *st_ptr;
    while (!st.stop_capture.load()) {
        try {
            auto frames = st.pipe.wait_for_frames(1000);
            auto color = frames.get_color_frame();
            if (!color) {
                continue;
            }
            FrameRow row;
            row.host_monotonic_ns = monotonic_ns();
            row.system_time_ns = system_time_ns();
            row.rs_frame_number = static_cast<unsigned long long>(color.get_frame_number());
            row.rs_timestamp_ms = color.get_timestamp();
            row.pixel_count = color.get_width() * color.get_height();
            {
                std::lock_guard<std::mutex> lock(st.mu);
                row.frame_idx = st.frame_idx++;
                long long start_ns = st.record_start_host_ns.load();
                row.t_rel_s = start_ns > 0 ? (row.host_monotonic_ns - start_ns) / 1e9 : 0.0;
                st.frame_rows.push_back(row);
            }
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

static std::string handle_start(RecorderState& st, const std::string& body) {
    if (st.recording.load()) {
        return json_error("already recording");
    }
    std::string session_id = get_json_string(body, "session_id", "");
    if (session_id.empty()) {
        return json_error("missing session_id");
    }
    st.camera_label = get_json_string(body, "camera_label", "camera");
    std::string preferred = get_json_string(body, "camera_device_name", "L515");
    std::string out_dir = get_json_string(body, "camera_output_dir", "");
    std::string bag_dir = get_json_string(body, "camera_bag_output_dir", "");
    std::string ts_dir = get_json_string(body, "camera_frame_ts_output_dir", "");
    std::string meta_dir = get_json_string(body, "camera_metadata_output_dir", "");
    std::string intr_dir = get_json_string(body, "camera_intrinsics_output_dir", "");
    std::string frames_dir = get_json_string(body, "camera_frames_output_dir", "");
    std::string depth_csv_dir = get_json_string(body, "camera_depth_csv_output_dir", "");
    st.video_w = static_cast<int>(get_json_int(body, "camera_width", 640));
    st.video_h = static_cast<int>(get_json_int(body, "camera_height", 480));
    st.video_fps = static_cast<double>(get_json_int(body, "camera_fps", 30));
    int depth_w = static_cast<int>(get_json_int(body, "camera_depth_width", st.video_w));
    int depth_h = static_cast<int>(get_json_int(body, "camera_depth_height", st.video_h));
    int depth_fps = static_cast<int>(get_json_int(body, "camera_depth_fps", static_cast<long long>(st.video_fps)));
    st.record_start_host_ns.store(get_json_int(body, "record_start_host_ns", 0));
    st.depth_enabled = get_json_bool(body, "camera_record_depth", true);
    st.infra_enabled = get_json_bool(body, "camera_record_infra", false);
    st.bag_enabled = get_json_bool(body, "camera_save_bag", true);
    st.export_frame_every_n = std::max(1, static_cast<int>(get_json_int(body, "camera_export_frame_every_n", 1)));
    st.max_export_frames = std::max(0, static_cast<int>(get_json_int(body, "camera_export_max_frames", 0)));
    st.video_codec = "bag_deferred_postprocess";
    st.postprocess_required = false;
    {
        std::lock_guard<std::mutex> lock(st.mu);
        st.frame_rows.clear();
        st.frame_idx = 0;
    }
    st.video_path.clear();
    st.bag_path.clear();
    st.frame_ts_path.clear();
    st.metadata_path.clear();
    st.intrinsics_path.clear();
    st.frame_export_dir = frames_dir.empty() ? "" : (fs::path(frames_dir) / (st.camera_label + "_" + session_id)).string();
    st.depth_csv_dir = depth_csv_dir.empty() ? "" : (fs::path(depth_csv_dir) / (st.camera_label + "_" + session_id)).string();

    if (!out_dir.empty()) {
        fs::create_directories(out_dir);
        st.video_path = (fs::path(out_dir) / (st.camera_label + "_" + session_id + ".mp4")).string();
    }
    if (!bag_dir.empty()) {
        fs::create_directories(bag_dir);
        st.bag_path = (fs::path(bag_dir) / (st.camera_label + "_" + session_id + ".bag")).string();
    }
    if (!ts_dir.empty()) {
        fs::create_directories(ts_dir);
        st.frame_ts_path = (fs::path(ts_dir) / (st.camera_label + "_frames_" + session_id + ".csv")).string();
    }
    if (!meta_dir.empty()) {
        fs::create_directories(meta_dir);
        st.metadata_path = (fs::path(meta_dir) / (st.camera_label + "_metadata_" + session_id + ".json")).string();
    }
    if (!intr_dir.empty()) {
        fs::create_directories(intr_dir);
        st.intrinsics_path = (fs::path(intr_dir) / (st.camera_label + "_intrinsics_" + session_id + ".json")).string();
    }

    std::string err;
    if (!select_l515_device(st, preferred, err)) {
        return json_error(err);
    }

    try {
        rs2::config cfg;
        if (!st.serial.empty()) {
            cfg.enable_device(st.serial);
        }
        cfg.enable_stream(RS2_STREAM_COLOR, st.video_w, st.video_h, RS2_FORMAT_BGR8, static_cast<int>(st.video_fps));
        if (st.depth_enabled) {
            cfg.enable_stream(RS2_STREAM_DEPTH, depth_w, depth_h, RS2_FORMAT_Z16, depth_fps);
        }
        if (st.bag_enabled && !st.bag_path.empty()) {
            cfg.enable_record_to_file(st.bag_path);
            st.postprocess_required = true;
        }
        st.profile = st.pipe.start(cfg);
        if (!wait_for_first_color_frame(st, err)) {
            try {
                st.pipe.stop();
            } catch (...) {
            }
            remove_partial_bag(st);
            return json_error(std::string("pipeline start failed: ") + err);
        }
        st.pipeline_mode = st.bag_enabled ? "color+depth+bag_deferred_export" : "color+depth";
        try {
            auto device = st.profile.get_device();
            st.device_name = device.get_info(RS2_CAMERA_INFO_NAME);
            st.serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            for (auto&& sensor : device.query_sensors()) {
                if (sensor.supports(RS2_OPTION_DEPTH_UNITS)) {
                    st.depth_scale = sensor.get_option(RS2_OPTION_DEPTH_UNITS);
                    break;
                }
            }
        } catch (...) {
        }
        write_intrinsics_json(st);
        st.stop_capture.store(false);
        st.recording.store(true);
        st.capture_thread = std::thread(capture_loop, &st);
    } catch (const rs2::error& e) {
        return json_error(std::string("pipeline start failed: ") + e.what());
    } catch (const std::exception& e) {
        return json_error(std::string("pipeline start failed: ") + e.what());
    }

    std::ostringstream out;
    out << "{";
    out << kv("ok", "true", false) << ",";
    out << kv("recording", "true", false) << ",";
    out << kv("video_path", st.video_path) << ",";
    out << kv("bag_path", st.bag_path) << ",";
    out << kv("frame_ts_path", st.frame_ts_path) << ",";
    out << kv("metadata_path", st.metadata_path) << ",";
    out << kv("intrinsics_path", st.intrinsics_path) << ",";
    out << kv("frame_export_dir", st.frame_export_dir) << ",";
    out << kv("depth_csv_dir", st.depth_csv_dir) << ",";
    out << kv("serial", st.serial) << ",";
    out << kv("device_name", st.device_name) << ",";
    out << kv("video_codec", st.video_codec) << ",";
    out << kv("started_depth", st.depth_enabled ? "true" : "false", false) << ",";
    out << kv("started_infra", st.infra_enabled ? "true" : "false", false) << ",";
    out << kv("started_bag", st.bag_enabled ? "true" : "false", false) << ",";
    out << kv("pipeline_mode", st.pipeline_mode) << ",";
    out << kv("postprocess_required", st.postprocess_required ? "true" : "false", false) << ",";
    out << kv("camera_fps", std::to_string(static_cast<int>(st.video_fps)), false) << ",";
    out << kv("record_start_host_ns", std::to_string(st.record_start_host_ns.load()), false) << ",";
    out << kv("export_frame_every_n", std::to_string(st.export_frame_every_n), false) << ",";
    out << kv("export_max_frames", std::to_string(st.max_export_frames), false);
    out << "}\n";
    return out.str();
}

static std::string handle_stop(RecorderState& st) {
    if (st.recording.load()) {
        st.stop_capture.store(true);
        if (st.capture_thread.joinable()) {
            st.capture_thread.join();
        }
        try {
            st.pipe.stop();
        } catch (...) {
        }
        st.recording.store(false);
        st.pipeline_mode = "stopped";
        write_frame_ts_csv(st);
        write_metadata_json(st);
        write_intrinsics_json(st);
    }
    long long bag_size = 0;
    if (!st.bag_path.empty() && fs::exists(st.bag_path)) {
        bag_size = static_cast<long long>(fs::file_size(st.bag_path));
    }
    long long video_size = 0;
    if (!st.video_path.empty() && fs::exists(st.video_path)) {
        video_size = static_cast<long long>(fs::file_size(st.video_path));
    }
    const bool bag_ok = !st.bag_enabled || (!st.bag_path.empty() && bag_size > 1024);
    const bool frames_ok = !st.frame_rows.empty();
    std::ostringstream out;
    out << "{";
    out << kv("ok", (bag_ok && frames_ok) ? "true" : "false", false) << ",";
    if (!bag_ok || !frames_ok) {
        out << kv("error", "bag file not written or no frames captured") << ",";
    }
    out << kv("video_path", st.video_path) << ",";
    out << kv("video_size", std::to_string(video_size), false) << ",";
    out << kv("bag_path", st.bag_path) << ",";
    out << kv("frame_ts_path", st.frame_ts_path) << ",";
    out << kv("metadata_path", st.metadata_path) << ",";
    out << kv("intrinsics_path", st.intrinsics_path) << ",";
    out << kv("frame_export_dir", st.frame_export_dir) << ",";
    out << kv("depth_csv_dir", st.depth_csv_dir) << ",";
    out << kv("frame_count", std::to_string(st.frame_rows.size()), false) << ",";
    out << kv("device_name", st.device_name) << ",";
    out << kv("serial", st.serial) << ",";
    out << kv("camera_fps", std::to_string(static_cast<int>(st.video_fps)), false) << ",";
    out << kv("video_codec", st.video_codec) << ",";
    out << kv("postprocess_required", st.postprocess_required ? "true" : "false", false) << ",";
    out << kv("export_frame_every_n", std::to_string(st.export_frame_every_n), false) << ",";
    out << kv("export_max_frames", std::to_string(st.max_export_frames), false);
    out << "}\n";
    return out.str();
}

static std::string handle_mark_start(RecorderState& st, const std::string& body) {
    long long start_ns = get_json_int(body, "record_start_host_ns", 0);
    {
        std::lock_guard<std::mutex> lock(st.mu);
        st.record_start_host_ns.store(start_ns);
        st.frame_rows.clear();
        st.frame_idx = 0;
    }
    std::ostringstream out;
    out << "{";
    out << kv("ok", "true", false) << ",";
    out << kv("recording", st.recording ? "true" : "false", false) << ",";
    out << kv("record_start_host_ns", std::to_string(st.record_start_host_ns.load()), false) << ",";
    out << kv("frame_count", "0", false);
    out << "}\n";
    return out.str();
}

static std::string handle_request(RecorderState& st, const std::string& body) {
    std::string cmd = get_json_string(body, "cmd", "");
    if (cmd == "ping") {
        return json_ok_ping(st);
    }
    if (cmd == "status") {
        return json_status(st);
    }
    if (cmd == "start") {
        return handle_start(st, body);
    }
    if (cmd == "stop") {
        return handle_stop(st);
    }
    if (cmd == "mark_start") {
        return handle_mark_start(st, body);
    }
    if (cmd == "shutdown") {
        st.running.store(false);
        if (st.recording.load()) {
            handle_stop(st);
        }
        return "{"
               "\"ok\":true"
               "}\n";
    }
    return json_error("unknown cmd: " + cmd);
}

int main(int argc, char** argv) {
    std::signal(SIGPIPE, SIG_IGN);
    std::string host = "127.0.0.1";
    int port = 61338;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            host = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        }
    }

    int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "socket create failed\n";
        return 1;
    }
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = inet_addr(host.c_str());
    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind failed\n";
        close(server_fd);
        return 1;
    }
    if (listen(server_fd, 8) < 0) {
        std::cerr << "listen failed\n";
        close(server_fd);
        return 1;
    }

    RecorderState st;
    std::cout << "[camera-service-system] listening on " << host << ":" << port << std::endl;

    while (st.running.load()) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            continue;
        }
        std::string body;
        char buf[4096];
        while (true) {
            ssize_t n = recv(client_fd, buf, sizeof(buf), 0);
            if (n <= 0) {
                break;
            }
            body.append(buf, buf + n);
            if (!body.empty() && body.back() == '\n') {
                break;
            }
        }
        std::string resp = handle_request(st, body);
        send(client_fd, resp.data(), resp.size(), 0);
        close(client_fd);
    }

    close(server_fd);
    return 0;
}
