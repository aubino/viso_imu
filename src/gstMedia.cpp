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

