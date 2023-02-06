#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ASICamera2.h>

namespace py = pybind11;

static void BindEnums(py::module& m)
{
    py::enum_<ASI_BAYER_PATTERN>(m, "BAYER_PATTERN")
        .value("BAYER_RG", ASI_BAYER_RG)
        .value("BAYER_BG", ASI_BAYER_BG)
        .value("BAYER_GR", ASI_BAYER_GR)
        .value("BAYER_GB", ASI_BAYER_GB)
        .export_values();

    py::enum_<ASI_IMG_TYPE>(m, "IMG_TYPE")
        .value("IMG_RAW8", ASI_IMG_RAW8)
        .value("IMG_RGB24", ASI_IMG_RGB24)
        .value("IMG_RAW16", ASI_IMG_RAW16)
        .value("IMG_Y8", ASI_IMG_Y8)
        .value("IMG_END", ASI_IMG_END)
        .export_values();

    py::enum_<ASI_GUIDE_DIRECTION>(m, "GUIDE_DIRECTION")
        .value("GUIDE_NORTH", ASI_GUIDE_NORTH)
        .value("GUIDE_SOUTH", ASI_GUIDE_SOUTH)
        .value("GUIDE_EAST", ASI_GUIDE_EAST)
        .value("GUIDE_WEST", ASI_GUIDE_WEST)
        .export_values();

    py::enum_<ASI_FLIP_STATUS>(m, "FLIP_STATUS")
        .value("FLIP_NONE", ASI_FLIP_NONE)
        .value("FLIP_HORIZ", ASI_FLIP_HORIZ)
        .value("FLIP_VERT", ASI_FLIP_VERT)
        .value("FLIP_BOTH", ASI_FLIP_BOTH)
        .export_values();

    py::enum_<ASI_CAMERA_MODE>(m, "CAMERA_MODE")
        .value("MODE_NORMAL", ASI_MODE_NORMAL)
        .value("MODE_TRIG_SOFT_EDGE", ASI_MODE_TRIG_SOFT_EDGE)
        .value("MODE_TRIG_RISE_EDGE", ASI_MODE_TRIG_RISE_EDGE)
        .value("MODE_TRIG_FALL_EDGE", ASI_MODE_TRIG_FALL_EDGE)
        .value("MODE_TRIG_SOFT_LEVEL", ASI_MODE_TRIG_SOFT_LEVEL)
        .value("MODE_TRIG_HIGH_LEVEL", ASI_MODE_TRIG_HIGH_LEVEL)
        .value("MODE_TRIG_LOW_LEVEL", ASI_MODE_TRIG_LOW_LEVEL)
        .value("MODE_END", ASI_MODE_END)
        .export_values();

    py::enum_<ASI_TRIG_OUTPUT>(m, "TRIG_OUTPUT")
        .value("TRIG_OUTPUT_PINA", ASI_TRIG_OUTPUT_PINA)
        .value("TRIG_OUTPUT_PINB", ASI_TRIG_OUTPUT_PINB)
        .value("TRIG_OUTPUT_NONE", ASI_TRIG_OUTPUT_NONE)
        .export_values();

    // py::enum_<ASI_ERROR_CODE>(m, "ERROR_CODE")
    //     .value("SUCCESS", ASI_SUCCESS)
    //     .value("ERROR_INVALID_INDEX", ASI_ERROR_INVALID_INDEX)
    //     .value("ERROR_INVALID_ID", ASI_ERROR_INVALID_ID)
    //     .value("ERROR_INVALID_CONTROL_TYPE", ASI_ERROR_INVALID_CONTROL_TYPE)
    //     .value("ERROR_CAMERA_CLOSED", ASI_ERROR_CAMERA_CLOSED)
    //     .value("ERROR_CAMERA_REMOVED", ASI_ERROR_CAMERA_REMOVED)
    //     .value("ERROR_INVALID_PATH", ASI_ERROR_INVALID_PATH)
    //     .value("ERROR_INVALID_FILEFORMAT", ASI_ERROR_INVALID_FILEFORMAT)
    //     .value("ERROR_INVALID_SIZE", ASI_ERROR_INVALID_SIZE)
    //     .value("ERROR_INVALID_IMGTYPE", ASI_ERROR_INVALID_IMGTYPE)
    //     .value("ERROR_OUTOF_BOUNDARY", ASI_ERROR_OUTOF_BOUNDARY)
    //     .value("ERROR_TIMEOUT", ASI_ERROR_TIMEOUT)
    //     .value("ERROR_INVALID_SEQUENCE", ASI_ERROR_INVALID_SEQUENCE)
    //     .value("ERROR_BUFFER_TOO_SMALL", ASI_ERROR_BUFFER_TOO_SMALL)
    //     .value("ERROR_VIDEO_MODE_ACTIVE", ASI_ERROR_VIDEO_MODE_ACTIVE)
    //     .value("ERROR_EXPOSURE_IN_PROGRESS", ASI_ERROR_EXPOSURE_IN_PROGRESS)
    //     .value("ERROR_GENERAL_ERROR", ASI_ERROR_GENERAL_ERROR)
    //     .value("ERROR_INVALID_MODE", ASI_ERROR_INVALID_MODE)
    //     .value("ERROR_END", ASI_ERROR_END)
    //     .export_values();

    // py::enum_<ASI_BOOL>(m, "BOOL")
    //     .value("FALSE", ASI_FALSE)
    //     .value("TRUE", ASI_TRUE)
    //     .export_values();

    py::enum_<ASI_CONTROL_TYPE>(m, "CONTROL_TYPE")
        .value("GAIN", ASI_GAIN)
        .value("EXPOSURE", ASI_EXPOSURE)
        .value("GAMMA", ASI_GAMMA)
        .value("WB_R", ASI_WB_R)
        .value("WB_B", ASI_WB_B)
        .value("OFFSET", ASI_OFFSET)
        .value("BANDWIDTHOVERLOAD", ASI_BANDWIDTHOVERLOAD)
        .value("OVERCLOCK", ASI_OVERCLOCK)
        .value("TEMPERATURE", ASI_TEMPERATURE)
        .value("FLIP", ASI_FLIP)
        .value("AUTO_MAX_GAIN", ASI_AUTO_MAX_GAIN)
        .value("AUTO_MAX_EXP", ASI_AUTO_MAX_EXP)
        .value("AUTO_TARGET_BRIGHTNESS", ASI_AUTO_TARGET_BRIGHTNESS)
        .value("HARDWARE_BIN", ASI_HARDWARE_BIN)
        .value("HIGH_SPEED_MODE", ASI_HIGH_SPEED_MODE)
        .value("COOLER_POWER_PERC", ASI_COOLER_POWER_PERC)
        .value("TARGET_TEMP", ASI_TARGET_TEMP)
        .value("COOLER_ON", ASI_COOLER_ON)
        .value("MONO_BIN", ASI_MONO_BIN)
        .value("FAN_ON", ASI_FAN_ON)
        .value("PATTERN_ADJUST", ASI_PATTERN_ADJUST)
        .value("ANTI_DEW_HEATER", ASI_ANTI_DEW_HEATER)
        .export_values();

    py::enum_<ASI_EXPOSURE_STATUS>(m, "EXPOSURE_STATUS")
        .value("EXP_IDLE", ASI_EXP_IDLE)
        .value("EXP_WORKING", ASI_EXP_WORKING)
        .value("EXP_SUCCESS", ASI_EXP_SUCCESS)
        .value("EXP_FAILED", ASI_EXP_FAILED)
        .export_values();
}

#ifdef WIN32
struct AsiException : std::exception {
    explicit AsiException(const char* message)
        : std::exception(message)
    {
    }
};
#else
struct AsiException : std::exception {
    explicit AsiException(const char* message)
        : message_(message)
    {
    }

    virtual const char* what() const _GLIBCXX_TXN_SAFE_DYN _GLIBCXX_USE_NOEXCEPT
    {
        return message_;
    }

    const char* message_;
};
#endif // WIN32

static void BindStructs(py::module& m)
{
    py::class_<ASI_CAMERA_INFO, std::shared_ptr<ASI_CAMERA_INFO>>(m, "CAMERA_INFO")
        .def_readonly("name", &ASI_CAMERA_INFO::Name)
        .def_readonly("camera_id", &ASI_CAMERA_INFO::CameraID)
        .def_readonly("max_height", &ASI_CAMERA_INFO::MaxHeight)
        .def_readonly("max_width", &ASI_CAMERA_INFO::MaxWidth)
        // ASI_BOOL IsColorCam;
        .def_property_readonly("is_color_cam", [](const ASI_CAMERA_INFO& o) -> bool { return o.IsColorCam != ASI_FALSE; })
        // ASI_BAYER_PATTERN BayerPattern;
        .def_readonly("bayer_pattern", &ASI_CAMERA_INFO::BayerPattern)

        // int SupportedBins[16];
        .def_property_readonly("supported_bins", [](const ASI_CAMERA_INFO& o) -> std::vector<int> {
            std::vector<int> bins;
            for (int i = 0; i < 16; i++) {
                if (o.SupportedBins[i] == 0)
                    break;
                bins.push_back(o.SupportedBins[i]);
            }
            return bins;
        })

        // ASI_IMG_TYPE SupportedVideoFormat[8];
        .def_property_readonly("supported_video_format", [](const ASI_CAMERA_INFO& o) -> std::vector<ASI_IMG_TYPE> {
            std::vector<ASI_IMG_TYPE> format;
            for (int i = 0; i < 8; i++) {
                if (o.SupportedVideoFormat[i] == ASI_IMG_TYPE::ASI_IMG_END)
                    break;
                format.push_back(o.SupportedVideoFormat[i]);
            }
            return format;
        })

        .def_readonly("pixel_size", &ASI_CAMERA_INFO::PixelSize)
        // ASI_BOOL MechanicalShutter;
        .def_property_readonly("mechanical_shutter", [](const ASI_CAMERA_INFO& o) -> bool { return o.MechanicalShutter != ASI_FALSE; })
        // ASI_BOOL ST4Port;
        .def_property_readonly("st4_port", [](const ASI_CAMERA_INFO& o) -> bool { return o.ST4Port != ASI_FALSE; })
        // ASI_BOOL IsCoolerCam;
        .def_property_readonly("is_cooler_cam", [](const ASI_CAMERA_INFO& o) -> bool { return o.IsCoolerCam != ASI_FALSE; })
        // ASI_BOOL IsUSB3Host;
        .def_property_readonly("is_usb3_host", [](const ASI_CAMERA_INFO& o) -> bool { return o.IsUSB3Host != ASI_FALSE; })
        // ASI_BOOL IsUSB3Camera;
        .def_property_readonly("is_usb3_camera", [](const ASI_CAMERA_INFO& o) -> bool { return o.IsUSB3Camera != ASI_FALSE; })
        .def_readonly("elec_per_adu", &ASI_CAMERA_INFO::ElecPerADU)
        .def_readonly("bit_depth", &ASI_CAMERA_INFO::BitDepth)
        // ASI_BOOL IsTriggerCam;
        .def_property_readonly("is_trigger_cam", [](const ASI_CAMERA_INFO& o) -> bool { return o.IsTriggerCam != ASI_FALSE; });

    py::class_<ASI_CONTROL_CAPS, std::shared_ptr<ASI_CONTROL_CAPS>>(m, "CONTROL_CAPS")
        .def_readonly("name", &ASI_CONTROL_CAPS::Name)
        .def_readonly("description", &ASI_CONTROL_CAPS::Description)
        .def_readonly("max_value", &ASI_CONTROL_CAPS::MaxValue)
        .def_readonly("min_value", &ASI_CONTROL_CAPS::MinValue)
        .def_readonly("default_value", &ASI_CONTROL_CAPS::DefaultValue)
        // ASI_BOOL IsAutoSupported;
        .def_property_readonly("is_auto_supported", [](const ASI_CONTROL_CAPS& o) -> bool { return o.IsAutoSupported != ASI_FALSE; })
        // ASI_BOOL IsWritable;
        .def_property_readonly("is_writable", [](const ASI_CONTROL_CAPS& o) -> bool { return o.IsWritable != ASI_FALSE; })
        // ASI_CONTROL_TYPE ControlType;
        .def_readonly("control_type", &ASI_CONTROL_CAPS::ControlType);

    // py::class_<ASI_ID, std::shared_ptr<ASI_ID>>(m, "ID")
    // .def_readonly("id", &ASI_ID::id);

    // py::class_<ASI_SUPPORTED_MODE, std::shared_ptr<ASI_SUPPORTED_MODE>>(m, "SUPPORTED_MODE")
    // // ASI_CAMERA_MODE SupportedCameraMode;
    // .def_readonly("supported_camera_mode", &ASI_SUPPORTED_MODE::SupportedCameraMode);

    py::register_exception<AsiException>(m, "Exception");
}

static const char* errorStrings[] {
    "success",
    "invalid index",
    "invalid id",
    "invalid control type",
    "camera closed",
    "camera removed",
    "invalid path",
    "invalid file format",
    "invalid size",
    "invalid image type",
    "out of boundary",
    "timeout",
    "invalid sequence",
    "buffer too small",
    "video mode active",
    "exposure in progress",
    "general error",
    "invalid mode",
    "end"
};

/**
 * \brief 检查错误抛出异常
 */
inline void CheckError(const char* function, ASI_ERROR_CODE errorCode)
{
    if (errorCode == ASI_SUCCESS) {
        return;
    }

    static char message[512];
    sprintf(message, "Failed to call %s, error: %s.", function, errorStrings[errorCode]);

    throw AsiException(message);
}

/**
 * \brief 转换到 numpy array
 */
inline std::variant<py::array_t<uint8_t>, py::array_t<uint16_t>> ConvertToArray(int cameraID, unsigned char* buffer)
{
    int width;
    int height;
    int bin;
    ASI_IMG_TYPE imgType;
    ASIGetROIFormat(cameraID, &width, &height, &bin, &imgType);

    py::capsule handle(buffer, [](void* ptr) {});

    if (imgType == ASI_IMG_RAW8) {
        py::array_t<uint8_t> output({ height, width, 1 }, buffer, handle);
        return output;
    }

    if (imgType == ASI_IMG_RGB24) {
        py::array_t<uint8_t> output({ height, width, 3 }, buffer, handle);
        return output;
    }

    if (imgType == ASI_IMG_RAW16) {
        py::array_t<uint16_t> output({ height, width, 1 }, reinterpret_cast<uint16_t*>(buffer), handle);
        return output;
    }

    if (imgType == ASI_IMG_Y8) {
        py::array_t<uint8_t> output({ height, width, 1 }, buffer, handle);
        return output;
    }

    return {};
}

static void BindFunctions(py::module& m)
{
    // int ASIGetNumOfConnectedCameras();
    m.def("get_num_of_connected_cameras", &ASIGetNumOfConnectedCameras);

    // int ASIGetProductIDs(int* pPIDs);
    m.def("get_product_ids", []() -> std::vector<int> {
        std::vector<int> productIDs(1024);
        const int size = ASIGetProductIDs(productIDs.data());
        productIDs.resize(size);
        return productIDs;
    });

    // ASI_BOOL ASICameraCheck(int iVID, int iPID);
    m.def(
        "camera_check", [](int productID) -> bool {
            const int VID = 0x03C3;
            return ASICameraCheck(VID, productID) != ASI_FALSE;
        },
        py::arg("productID"));

    // ASI_ERROR_CODE ASIGetCameraProperty(ASI_CAMERA_INFO * pASICameraInfo, int iCameraIndex);
    m.def(
        "get_camera_property", [](int cameraIndex) -> std::shared_ptr<ASI_CAMERA_INFO> {
            auto cameraInfo = std::make_shared<ASI_CAMERA_INFO>();
            const auto errorCode = ASIGetCameraProperty(cameraInfo.get(), cameraIndex);
            CheckError("get_camera_property", errorCode);

            return cameraInfo;
        },
        py::arg("cameraIndex"));

    // ASI_ERROR_CODE ASIGetCameraPropertyByID(int iCameraID, ASI_CAMERA_INFO* pASICameraInfo);
    m.def(
        "get_camera_property_by_id", [](int cameraID) -> std::shared_ptr<ASI_CAMERA_INFO> {
            auto cameraInfo = std::make_shared<ASI_CAMERA_INFO>();
            const auto errorCode = ASIGetCameraPropertyByID(cameraID, cameraInfo.get());
            CheckError("get_camera_property_by_id", errorCode);

            return cameraInfo;
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIOpenCamera(int iCameraID);
    m.def(
        "open_camera", [](int cameraID) {
            const auto errorCode = ASIOpenCamera(cameraID);
            CheckError("open_camera", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIInitCamera(int iCameraID);
    m.def(
        "init_camera", [](int cameraID) {
            const auto errorCode = ASIInitCamera(cameraID);
            CheckError("init_camera", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASICloseCamera(int iCameraID);
    m.def(
        "close_camera", [](int cameraID) {
            const auto errorCode = ASICloseCamera(cameraID);
            CheckError("close_camera", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetNumOfControls(int iCameraID, int* piNumberOfControls);
    m.def(
        "get_num_of_controls", [](int cameraID) -> int {
            int numberOfControls;
            const auto errorCode = ASIGetNumOfControls(cameraID, &numberOfControls);
            CheckError("get_num_of_controls", errorCode);

            return numberOfControls;
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetControlCaps(int iCameraID, int iControlIndex, ASI_CONTROL_CAPS* pControlCaps);
    m.def(
        "get_control_caps", [](int cameraID, int controlIndex) -> std::shared_ptr<ASI_CONTROL_CAPS> {
            auto controlCaps = std::make_shared<ASI_CONTROL_CAPS>();
            const auto errorCode = ASIGetControlCaps(cameraID, controlIndex, controlCaps.get());
            CheckError("get_control_caps", errorCode);

            return controlCaps;
        },
        py::arg("cameraID"), py::arg("controlIndex"));

    // ASI_ERROR_CODE ASIGetControlValue(int iCameraID, ASI_CONTROL_TYPE ControlType, long* plValue, ASI_BOOL* pbAuto);
    m.def(
        "get_control_value", [](int cameraID, ASI_CONTROL_TYPE controlType) -> std::pair<long, bool> {
            long value;
            ASI_BOOL bAuto;
            const auto errorCode = ASIGetControlValue(cameraID, controlType, &value, &bAuto);
            CheckError("get_control_value", errorCode);

            return std::make_pair(value, bAuto != ASI_FALSE);
        },
        py::arg("cameraID"), py::arg("ASI_CONTROL_TYPE"));

    // ASI_ERROR_CODE ASISetControlValue(int iCameraID, ASI_CONTROL_TYPE ControlType, long lValue, ASI_BOOL bAuto);
    m.def(
        "set_control_value", [](int cameraID, ASI_CONTROL_TYPE controlType, long value, bool bAuto) {
            const auto errorCode = ASISetControlValue(cameraID, controlType, value, bAuto ? ASI_TRUE : ASI_FALSE);
            CheckError("set_control_value", errorCode);
        },
        py::arg("cameraID"), py::arg("controlType"), py::arg("value"), py::arg("bAuto"));

    // ASI_ERROR_CODE ASISetROIFormat(int iCameraID, int iWidth, int iHeight, int iBin, ASI_IMG_TYPE Img_type);
    m.def(
        "set_roi_format", [](int cameraID, int width, int height, int bin, ASI_IMG_TYPE imgType) {
            const auto errorCode = ASISetROIFormat(cameraID, width, height, bin, imgType);
            CheckError("set_roi_format", errorCode);
        },
        py::arg("cameraID"), py::arg("width"), py::arg("height"), py::arg("bin"), py::arg("imgType"));

    // ASI_ERROR_CODE ASIGetROIFormat(int iCameraID, int* piWidth, int* piHeight, int* piBin, ASI_IMG_TYPE* pImg_type);
    m.def(
        "get_roi_format", [](int cameraID) -> std::tuple<int, int, int, ASI_IMG_TYPE> {
            int width;
            int height;
            int bin;
            ASI_IMG_TYPE imgType;
            const auto errorCode = ASIGetROIFormat(cameraID, &width, &height, &bin, &imgType);
            CheckError("get_roi_format", errorCode);

            return std::make_tuple(width, height, bin, imgType);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASISetStartPos(int iCameraID, int iStartX, int iStartY);
    m.def(
        "set_start_pos", [](int cameraID, int startX, int startY) {
            const auto errorCode = ASISetStartPos(cameraID, startX, startY);
            CheckError("set_start_pos", errorCode);
        },
        py::arg("cameraID"), py::arg("startX"), py::arg("startY"));

    // ASI_ERROR_CODE ASIGetStartPos(int iCameraID, int* piStartX, int* piStartY);
    m.def(
        "get_start_pos", [](int cameraID) -> std::pair<int, int> {
            int startX;
            int startY;
            const auto errorCode = ASIGetStartPos(cameraID, &startX, &startY);
            CheckError("get_start_pos", errorCode);

            return std::make_pair(startX, startY);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetDroppedFrames(int iCameraID, int* piDropFrames);
    m.def(
        "get_dropped_frames", [](int cameraID) -> int {
            int dropFrames;
            const auto errorCode = ASIGetDroppedFrames(cameraID, &dropFrames);
            CheckError("get_dropped_frames", errorCode);

            return dropFrames;
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIEnableDarkSubtract(int iCameraID, char* pcBMPPath);
    m.def(
        "enable_dark_subtract", [](int cameraID, char* bmpPath) {
            const auto errorCode = ASIEnableDarkSubtract(cameraID, bmpPath);
            CheckError("enable_dark_subtract", errorCode);
        },
        py::arg("cameraID"), py::arg("bmpPath"));

    // ASI_ERROR_CODE ASIDisableDarkSubtract(int iCameraID);
    m.def(
        "disable_dark_subtract", [](int cameraID) {
            const auto errorCode = ASIDisableDarkSubtract(cameraID);
            CheckError("disable_dark_subtract", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIStartVideoCapture(int iCameraID);
    m.def(
        "start_video_capture", [](int cameraID) {
            const auto errorCode = ASIStartVideoCapture(cameraID);
            CheckError("start_video_capture", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIStopVideoCapture(int iCameraID);
    m.def(
        "stop_video_capture", [](int cameraID) {
            const auto errorCode = ASIStopVideoCapture(cameraID);
            CheckError("stop_video_capture", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetVideoData(int iCameraID, unsigned char* pBuffer, long lBuffSize, int iWaitms);
    m.def(
        "get_video_data", [](int cameraID, int waitMs) -> std::variant<py::array_t<uint8_t>, py::array_t<uint16_t>> {
            constexpr int HEIGHT = 960;
            constexpr int WIDTH = 1280;
            constexpr int CHANNEL = 3;

            constexpr int BUFFER_SIZE = HEIGHT * WIDTH * CHANNEL;
            static unsigned char buffer[BUFFER_SIZE];

            const auto errorCode = ASIGetVideoData(cameraID, buffer, BUFFER_SIZE, waitMs);
            CheckError("get_video_data", errorCode);

            return ConvertToArray(cameraID, buffer);
        },
        py::arg("cameraID"), py::arg("waitMs"));

    // ASI_ERROR_CODE ASIPulseGuideOn(int iCameraID, ASI_GUIDE_DIRECTION direction);
    m.def(
        "pulse_guide_on", [](int cameraID, ASI_GUIDE_DIRECTION direction) {
            const auto errorCode = ASIPulseGuideOn(cameraID, direction);
            CheckError("pulse_guide_on", errorCode);
        },
        py::arg("cameraID"), py::arg("direction"));

    // ASI_ERROR_CODE ASIPulseGuideOff(int iCameraID, ASI_GUIDE_DIRECTION direction);
    m.def(
        "pulse_guide_off", [](int cameraID, ASI_GUIDE_DIRECTION direction) {
            const auto errorCode = ASIPulseGuideOff(cameraID, direction);
            CheckError("pulse_guide_off", errorCode);
        },
        py::arg("cameraID"), py::arg("direction"));

    // ASI_ERROR_CODE ASIStartExposure(int iCameraID, ASI_BOOL bIsDark);
    m.def(
        "start_exposure", [](int cameraID, bool isDark) {
            const auto errorCode = ASIStartExposure(cameraID, isDark ? ASI_TRUE : ASI_FALSE);
            CheckError("start_exposure", errorCode);
        },
        py::arg("cameraID"), py::arg("isDark"));

    // ASI_ERROR_CODE ASIStopExposure(int iCameraID);
    m.def(
        "stop_exposure", [](int cameraID) {
            const auto errorCode = ASIStopExposure(cameraID);
            CheckError("stop_exposure", errorCode);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetExpStatus(int iCameraID, ASI_EXPOSURE_STATUS* pExpStatus);
    m.def(
        "get_exp_status", [](int cameraID) -> ASI_EXPOSURE_STATUS {
            ASI_EXPOSURE_STATUS status;
            const auto errorCode = ASIGetExpStatus(cameraID, &status);
            CheckError("get_exp_status", errorCode);

            return status;
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetDataAfterExp(int iCameraID, unsigned char* pBuffer, long lBuffSize);
    m.def(
        "get_data_after_exp", [](int cameraID) -> std::variant<py::array_t<uint8_t>, py::array_t<uint16_t>> {
            constexpr int HEIGHT = 960;
            constexpr int WIDTH = 1280;
            constexpr int CHANNEL = 3;
            constexpr int BUFFER_SIZE = HEIGHT * WIDTH * CHANNEL;

            static unsigned char buffer[BUFFER_SIZE];
            const auto errorCode = ASIGetDataAfterExp(cameraID, buffer, BUFFER_SIZE);
            CheckError("get_data_after_exp", errorCode);

            return ConvertToArray(cameraID, buffer);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetID(int iCameraID, ASI_ID* pID);
    m.def(
        "get_id", [](int cameraID) -> std::vector<unsigned char> {
            ASI_ID id;
            const auto errorCode = ASIGetID(cameraID, &id);
            CheckError("get_id", errorCode);

            return std::vector<unsigned char> { &id.id[0], &id.id[0] + 8 };
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASISetID(int iCameraID, ASI_ID ID);
    m.def(
        "set_id", [](int cameraID, const std::vector<unsigned char>& ids) {
            ASI_ID id;
            for (int i = 0; i < ids.size() && i < 8; i++) {
                id.id[i] = ids[i];
            }

            const auto errorCode = ASISetID(cameraID, id);
            CheckError("set_id", errorCode);
        },
        py::arg("cameraID"), py::arg("ids"));

    // ASI_ERROR_CODE ASIGetGainOffset(int iCameraID, int* pOffset_HighestDR, int* pOffset_UnityGain, int* pGain_LowestRN, int* pOffset_LowestRN);
    m.def(
        "get_gain_offset", [](int cameraID) -> std::tuple<int, int, int, int> {
            int offsetHighestDR;
            int offsetUnityGain;
            int gainLowestRN;
            int offsetLowestRN;
            const auto errorCode = ASIGetGainOffset(cameraID, &offsetHighestDR, &offsetUnityGain, &gainLowestRN, &offsetLowestRN);
            CheckError("get_gain_offset", errorCode);

            return std::make_tuple(offsetHighestDR, offsetUnityGain, gainLowestRN, offsetLowestRN);
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetLMHGainOffset(int iCameraID, int* pLGain, int* pMGain, int* pHGain, int* pHOffset);
    m.def(
        "get_lmh_gain_offset", [](int cameraID) -> std::tuple<int, int, int, int> {
            int lGain;
            int mGain;
            int hGain;
            int hOffset;
            const auto errorCode = ASIGetLMHGainOffset(cameraID, &lGain, &mGain, &hGain, &hOffset);
            CheckError("get_lmh_gain_offset", errorCode);

            return std::make_tuple(lGain, mGain, hGain, hOffset);
        },
        py::arg("cameraID"));

    // char* ASIGetSDKVersion();
    m.def("get_sdk_version", &ASIGetSDKVersion);

    // ASI_ERROR_CODE ASIGetCameraSupportMode(int iCameraID, ASI_SUPPORTED_MODE* pSupportedMode);
    m.def(
        "get_camera_support_mode", [](int cameraID) -> std::vector<int> {
            ASI_SUPPORTED_MODE supportedMode;
            const auto errorCode = ASIGetCameraSupportMode(cameraID, &supportedMode);
            CheckError("get_camera_support_mode", errorCode);

            std::vector<int> mode;
            for (int i = 0; i < 16; i++) {
                if (supportedMode.SupportedCameraMode[i] == ASI_MODE_END)
                    break;
                mode.push_back(supportedMode.SupportedCameraMode[i]);
            }

            return mode;
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASIGetCameraMode(int iCameraID, ASI_CAMERA_MODE* mode);
    m.def(
        "get_camera_mode", [](int cameraID) -> ASI_CAMERA_MODE {
            ASI_CAMERA_MODE mode;
            const auto errorCode = ASIGetCameraMode(cameraID, &mode);
            CheckError("get_camera_mode", errorCode);

            return mode;
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASISetCameraMode(int iCameraID, ASI_CAMERA_MODE mode);
    m.def(
        "set_camera_mode", [](int cameraID, ASI_CAMERA_MODE mode) {
            const auto errorCode = ASISetCameraMode(cameraID, mode);
            CheckError("set_camera_mode", errorCode);
        },
        py::arg("cameraID"), py::arg("mode"));

    // ASI_ERROR_CODE ASISendSoftTrigger(int iCameraID, ASI_BOOL bStart);
    m.def(
        "send_soft_trigger", [](int cameraID, bool start) {
            const auto errorCode = ASISendSoftTrigger(cameraID, start ? ASI_TRUE : ASI_FALSE);
            CheckError("send_soft_trigger", errorCode);
        },
        py::arg("cameraID"), py::arg("start"));

    // ASI_ERROR_CODE ASIGetSerialNumber(int iCameraID, ASI_SN* pSN);
    m.def(
        "get_serial_number", [](int cameraID) -> std::vector<unsigned char> {
            ASI_SN SN;
            const auto errorCode = ASIGetSerialNumber(cameraID, &SN);
            CheckError("get_serial_number", errorCode);

            return std::vector<unsigned char> { SN.id, SN.id + 8 };
        },
        py::arg("cameraID"));

    // ASI_ERROR_CODE ASISetTriggerOutputIOConf(int iCameraID, ASI_TRIG_OUTPUT_PIN pin, ASI_BOOL bPinHigh, long lDelay, long lDuration);
    m.def(
        "set_trigger_output_io_conf", [](int cameraID, ASI_TRIG_OUTPUT_PIN pin, bool pinHigh, long delay, long duration) {
            const auto errorCode = ASISetTriggerOutputIOConf(cameraID, pin, pinHigh ? ASI_TRUE : ASI_FALSE, delay, duration);
            CheckError("set_trigger_output_io_conf", errorCode);
        },
        py::arg("cameraID"), py::arg("pin"), py::arg("pinHigh"), py::arg("delay"), py::arg("duration"));

    // ASI_ERROR_CODE ASIGetTriggerOutputIOConf(int iCameraID, ASI_TRIG_OUTPUT_PIN pin, ASI_BOOL* bPinHigh, long* lDelay, long* lDuration);
    m.def(
        "get_trigger_output_io_conf", [](int cameraID, ASI_TRIG_OUTPUT_PIN pin) -> std::tuple<bool, long, long> {
            ASI_BOOL pinHigh;
            long delay;
            long duration;
            const auto errorCode = ASIGetTriggerOutputIOConf(cameraID, pin, &pinHigh, &delay, &duration);
            CheckError("get_trigger_output_io_conf", errorCode);

            return std::make_tuple(pinHigh != ASI_FALSE, duration, duration);
        },
        py::arg("cameraID"), py::arg("pin"));
}

PYBIND11_MODULE(ASICamera, m)
{
    m.doc() = R"pbdoc(ASICamera for Python)pbdoc";
    m.attr("__version__") = "1.0.0";

    BindEnums(m);
    BindStructs(m);
    BindFunctions(m);
}
