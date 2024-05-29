#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <string>
#include <vector>

class RegHandler {
public:
    RegHandler(const std::string &filename);
    ~RegHandler();
    
    bool start();
    void stop();
    void feed(const std::vector<uint8_t> &h265Data);

private:
    std::string filename;
    GstElement *pipeline;
    GstElement *appsrc;
    GstBus *bus;
    bool isRunning;

    void unrefGstElements();
};

RegHandler::RegHandler(const std::string &filename)
    : filename(filename), pipeline(nullptr), appsrc(nullptr), bus(nullptr), isRunning(false) {
    gst_init(nullptr, nullptr);
}

RegHandler::~RegHandler() {
    if (isRunning) {
        stop();
    }
    unrefGstElements();
}

bool RegHandler::start() {
    if (isRunning) {
        std::cerr << "Pipeline is already running." << std::endl;
        return false;
    }

    // Define GStreamer pipeline string
    std::string pipeline_str = "appsrc name=mysource ! h265parse ! mp4mux ! filesink location=" + filename;
    GError *error = nullptr;
    pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
        std::cerr << "Failed to parse launch: " << error->message << std::endl;
        g_error_free(error);
        return false;
    }

    // Get the appsrc element
    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "mysource");

    // Set appsrc properties
    g_object_set(G_OBJECT(appsrc), "caps",
                 gst_caps_new_simple("video/x-h265",
                                     "stream-format", G_TYPE_STRING, "byte-stream",
                                     "alignment", G_TYPE_STRING, "au",
                                     NULL), NULL);
    g_object_set(G_OBJECT(appsrc), "format", GST_FORMAT_TIME, NULL);

    // Start the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Get the bus to handle messages
    bus = gst_element_get_bus(pipeline);

    isRunning = true;
    return true;
}

void RegHandler::stop() {
    if (!isRunning) {
        std::cerr << "Pipeline is not running." << std::endl;
        return;
    }

    // Signal end of stream
    gst_app_src_end_of_stream(GST_APP_SRC(appsrc));

    // Wait until error or EOS
    GstMessage *msg;
    gboolean terminate = FALSE;
    while (!terminate) {
        msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                         static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg != nullptr) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError *err;
                    gchar *debug_info;
                    gst_message_parse_error(msg, &err, &debug_info);
                    std::cerr << "Error received from element " << GST_OBJECT_NAME(msg->src) << ": " << err->message << std::endl;
                    std::cerr << "Debugging information: " << (debug_info ? debug_info : "none") << std::endl;
                    g_clear_error(&err);
                    g_free(debug_info);
                    terminate = TRUE;
                    break;
                }
                case GST_MESSAGE_EOS:
                    std::cout << "End-Of-Stream reached." << std::endl;
                    terminate = TRUE;
                    break;
                default:
                    std::cerr << "Unexpected message received." << std::endl;
                    break;
            }
            gst_message_unref(msg);
        }
    }

    // Stop the pipeline
    gst_element_set_state(pipeline, GST_STATE_NULL);
    isRunning = false;
}

void RegHandler::feed(const std::vector<uint8_t> &h265Data) {
    if (!isRunning) {
        std::cerr << "Pipeline is not running. Cannot feed data." << std::endl;
        return;
    }

    // Create a new GstBuffer to hold the H.265 data
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, h265Data.size(), nullptr);
    gst_buffer_fill(buffer, 0, h265Data.data(), h265Data.size());

    // Push the buffer into the appsrc
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
        std::cerr << "Error pushing buffer to appsrc" << std::endl;
    }
}

void RegHandler::unrefGstElements() {
    if (bus) {
        gst_object_unref(bus);
        bus = nullptr;
    }
    if (appsrc) {
        gst_object_unref(appsrc);
        appsrc = nullptr;
    }
    if (pipeline) {
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
}


////////////////////////////////////// Using FFMPEG ////////////////////////////////////////////////////////////////
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
}

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>

class RegHandler {
public:
    RegHandler(const std::string &filename);
    ~RegHandler();
    
    bool start();
    void stop();
    void feed(const std::vector<uint8_t> &h265Data);

private:
    std::string filename;
    AVFormatContext *formatContext;
    AVStream *videoStream;
    int64_t pts;
    bool isRunning;

    void init_ffmpeg();
    void cleanup_ffmpeg();
};

RegHandler::RegHandler(const std::string &filename)
    : filename(filename), formatContext(nullptr), videoStream(nullptr), pts(0), isRunning(false) {
    av_register_all();
    avcodec_register_all();
    avformat_network_init();
}

RegHandler::~RegHandler() {
    stop();
    cleanup_ffmpeg();
}

void RegHandler::init_ffmpeg() {
    avformat_alloc_output_context2(&formatContext, nullptr, "mp4", filename.c_str());
    if (!formatContext) {
        throw std::runtime_error("Could not create output context");
    }

    AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H265);
    if (!codec) {
        throw std::runtime_error("Codec not found");
    }

    videoStream = avformat_new_stream(formatContext, codec);
    if (!videoStream) {
        throw std::runtime_error("Failed allocating output stream");
    }

    AVCodecContext *codecContext = avcodec_alloc_context3(codec);
    if (!codecContext) {
        throw std::runtime_error("Could not allocate codec context");
    }

    codecContext->codec_id = AV_CODEC_ID_H265;
    codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
    codecContext->time_base = {1, 30}; // 30 FPS
    codecContext->framerate = {30, 1};
    codecContext->gop_size = 12;
    codecContext->pix_fmt = AV_PIX_FMT_YUV420P;

    if (formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
        codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    avcodec_parameters_from_context(videoStream->codecpar, codecContext);

    if (avio_open(&formatContext->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0) {
        throw std::runtime_error("Could not open output file");
    }

    if (avformat_write_header(formatContext, nullptr) < 0) {
        throw std::runtime_error("Error occurred when opening output file");
    }
}

void RegHandler::cleanup_ffmpeg() {
    if (formatContext) {
        av_write_trailer(formatContext);
        if (!(formatContext->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&formatContext->pb);
        }
        avformat_free_context(formatContext);
    }
}

bool RegHandler::start() {
    if (isRunning) {
        std::cerr << "Handler is already running." << std::endl;
        return false;
    }

    try {
        init_ffmpeg();
        isRunning = true;
    } catch (const std::exception &e) {
        std::cerr << "Failed to start RegHandler: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void RegHandler::stop() {
    if (!isRunning) {
        std::cerr << "Handler is not running." << std::endl;
        return;
    }

    cleanup_ffmpeg();
    isRunning = false;
}

void RegHandler::feed(const std::vector<uint8_t> &h265Data) {
    if (!isRunning) {
        std::cerr << "Handler is not running. Cannot feed data." << std::endl;
        return;
    }

    AVPacket packet;
    av_init_packet(&packet);
    packet.data = (uint8_t *)h265Data.data();
    packet.size = h265Data.size();
    packet.stream_index = videoStream->index;
    packet.pts = pts;
    packet.dts = pts;
    packet.duration = av_rescale_q(1, {1, 30}, videoStream->time_base);
    pts += packet.duration;

    if (av_interleaved_write_frame(formatContext, &packet) < 0) {
        std::cerr << "Error while writing video frame" << std::endl;
    }
}

int main() {
    RegHandler regHandler("output.mp4");

    if (!regHandler.start()) {
        return -1;
    }

    // Simulate receiving H.265 packets
    for (int i = 0; i < 100; ++i) {
        std::vector<uint8_t> h265Packet(1024); // Placeholder for H.265 data
        std::fill(h265Packet.begin(), h265Packet.end(), rand() % 256); // Simulate H.265 data
        regHandler.feed(h265Packet);
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Simulate 30 FPS
    }

    regHandler.stop();

    return 0;
}


