class DisplayPipeline {
public:
    DisplayPipeline(int width, int height) {
        // Define the pipeline string with configurable resolution
        std::string pipeline_desc =
            "appsrc name=appsrc ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), width=" + std::to_string(width) +
            ", height=" + std::to_string(height) +
            ", format=NV12 ! "
            "nvdrmvideosink";

        GError *error = nullptr;
        pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);
        if (!pipeline) {
            std::cerr << "Failed to create display pipeline: " << error->message << std::endl;
            g_clear_error(&error);
        }

        appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "appsrc");
    }

    ~DisplayPipeline() {
        stop();
        if (appsrc) gst_object_unref(appsrc);
        if (pipeline) gst_object_unref(pipeline);
    }

    void start() {
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        std::cout << "Display pipeline started\n";
    }

    void stop() {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        std::cout << "Display pipeline stopped\n";
    }

    void push_frame(GstSample *sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        gst_app_src_push_buffer(GST_APP_SRC(appsrc), gst_buffer_ref(buffer));
    }

private:
    GstElement *pipeline = nullptr, *appsrc = nullptr;
};
