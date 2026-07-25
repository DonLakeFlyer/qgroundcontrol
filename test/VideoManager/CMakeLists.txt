# ============================================================================
# VideoManager Tests
# ============================================================================

if(QGC_ENABLE_GST_VIDEOSTREAMING)
    add_subdirectory(GStreamer)
endif()

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        VideoManagerInitTest.cc
        VideoManagerInitTest.h
)

add_qgc_test(VideoManagerInitTest LABELS Unit)
