// Inside HdmiHandler class definition
GstElement *nvvidconv;

// Inside start() method after creating nvdrmvideosink
nvvidconv = gst_element_factory_make("nvvidconv", "nvvidconv");
if (!nvvidconv) {
    std::cerr << "Failed to create nvvidconv element." << std::endl;
    cleanup();
    return false;
}
gst_bin_add(GST_BIN(pipeline), nvvidconv);
if (!gst_element_link(appsrc, nvvidconv)) {
    std::cerr << "Failed to link appsrc and nvvidconv elements." << std::endl;
    cleanup();
    return false;
}
if (!gst_element_link(nvvidconv, nvdrmvideosink)) {
    std::cerr << "Failed to link nvvidconv and nvdrmvideosink elements." << std::endl;
    cleanup();
    return false;
}

// Inside feed() method before pushing buffer to appsrc
if (!gst_app_src_set_caps(GST_APP_SRC(appsrc), gst_caps_from_string("video/x-raw, format=RGBA"))) {
    std::cerr << "Failed to set caps for appsrc." << std::endl;
    return;
}
