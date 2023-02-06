import ASICamera as asi
import cv2
import time

print_ = print
def print(*args, **kwargs):
    print_('-' * 64)
    print_(*args, **kwargs)

print(__file__)

# help(asi.get_num_of_connected_cameras)
# help(asi.get_product_ids)
# help(asi.camera_check)
# help(asi.get_camera_property)
# help(asi.get_camera_property_by_id)
# help(asi.open_camera)
# help(asi.init_camera)
# help(asi.close_camera)
# help(asi.get_num_of_controls)
# help(asi.get_control_caps)
# help(asi.get_control_value)
# help(asi.set_control_value)
# help(asi.set_roi_format)
# help(asi.get_roi_format)
# help(asi.set_start_pos)
# help(asi.get_start_pos)
# help(asi.get_dropped_frames)
# help(asi.enable_dark_subtract)
# help(asi.disable_dark_subtract)
# help(asi.start_video_capture)
# help(asi.stop_video_capture)
# help(asi.get_video_data)
# help(asi.pulse_guide_on)
# help(asi.pulse_guide_off)
# help(asi.start_exposure)
# help(asi.stop_exposure)
# help(asi.get_exp_status)
# help(asi.get_data_after_exp)
# help(asi.get_id)
# help(asi.set_id)
# help(asi.get_gain_offset)
# help(asi.get_lmh_gain_offset)
# help(asi.get_sdk_version)
# help(asi.get_camera_support_mode)
# help(asi.get_camera_mode)
# help(asi.set_camera_mode)
# help(asi.send_soft_trigger)
# help(asi.get_serial_number)
# help(asi.set_trigger_output_io_conf)
# help(asi.get_trigger_output_io_conf)

def print_camera_info(camera_info):
    print_('-' * 64)
    print_("    camera_info.name                   = ", camera_info.name)
    print_("    camera_info.camera_id              = ", camera_info.camera_id)
    print_("    camera_info.max_height             = ", camera_info.max_height)
    print_("    camera_info.max_width              = ", camera_info.max_width)
    print_("    camera_info.is_color_cam           = ", camera_info.is_color_cam)
    print_("    camera_info.bayer_pattern          = ", camera_info.bayer_pattern)
    print_("    camera_info.supported_bins         = ", camera_info.supported_bins)
    print_("    camera_info.supported_video_format = ", camera_info.supported_video_format)
    print_("    camera_info.pixel_size             = ", camera_info.pixel_size)
    print_("    camera_info.mechanical_shutter     = ", camera_info.mechanical_shutter)
    print_("    camera_info.st4_port               = ", camera_info.st4_port)
    print_("    camera_info.is_cooler_cam          = ", camera_info.is_cooler_cam)
    print_("    camera_info.is_usb3_host           = ", camera_info.is_usb3_host)
    print_("    camera_info.is_usb3_camera         = ", camera_info.is_usb3_camera)
    print_("    camera_info.elec_per_adu           = ", camera_info.elec_per_adu)
    print_("    camera_info.bit_depth              = ", camera_info.bit_depth)
    print_("    camera_info.is_trigger_cam         = ", camera_info.is_trigger_cam)

def print_control_caps(control_caps):
    print_('-' * 64)
    print_('control_caps.name              = ', control_caps.name)
    print_('control_caps.description       = ', control_caps.description)
    print_('control_caps.max_value         = ', control_caps.max_value)
    print_('control_caps.min_value         = ', control_caps.min_value)
    print_('control_caps.default_value     = ', control_caps.default_value)
    print_('control_caps.is_auto_supported = ', control_caps.is_auto_supported)
    print_('control_caps.is_writable       = ', control_caps.is_writable)
    print_('control_caps.control_type      = ', control_caps.control_type)


try:
    num_of_connected_cameras = asi.get_num_of_connected_cameras()
    print('num_of_connected_cameras =', num_of_connected_cameras)

    product_ids = asi.get_product_ids()
    print('product_ids =', product_ids)

    for product_id in product_ids:
        print('camera_check = ', asi.camera_check(product_id))
        break

    camera_id = 0
    
    asi.open_camera(camera_id)
    asi.init_camera(camera_id)

    camera_info = asi.get_camera_property(camera_id)
    print_camera_info(camera_info)

    camera_info = asi.get_camera_property_by_id(camera_id)
    
    num_of_controls = asi.get_num_of_controls(camera_id)
    print('get_num_of_controls =', num_of_controls)

  
    for control_idx in range(num_of_controls):
        control_caps = asi.get_control_caps(camera_id, control_idx)
        print_control_caps(control_caps)

        control_value = asi.get_control_value(camera_id, control_caps.control_type)
        print_('control_value                  = ', control_value)
        break

    asi.set_roi_format(camera_id, 1280, 960, 1, asi.IMG_RGB24)

    roi_format = asi.get_roi_format(camera_id)
    print('get_roi_format =', roi_format)

    asi.set_start_pos(camera_id, 0, 0)

    start_pos = asi.get_start_pos(camera_id)
    print('start_pos =', start_pos)

    dropped_frames = asi.get_dropped_frames(camera_id)
    print('get_dropped_frames =', dropped_frames)

    # asi.enable_dark_subtract(camera_id, "./bmp")
    # asi.disable_dark_subtract(camera_id)

    test_video_capture = False

    if test_video_capture:
        asi.start_video_capture(camera_id)

        video_data = asi.get_video_data(camera_id, 100)
        print("video_data.dtype =", video_data.dtype, ", video_data.shape =", video_data.shape)
        
        cv2.imshow('get_video_data', video_data)
        cv2.waitKey(5000)

        asi.stop_video_capture(camera_id)

    test_exposure = True

    if test_exposure:
        asi.set_control_value(camera_id, asi.EXPOSURE, 100000, True)

        asi.start_exposure(camera_id, True)

        while asi.get_exp_status(camera_id) == asi.EXP_WORKING:
            time.sleep(0.001)

        asi.stop_exposure(camera_id)
        
        data_after_exp = asi.get_data_after_exp(camera_id)
        print("data_after_exp.dtype =", data_after_exp.dtype, ", data_after_exp.shape =", data_after_exp.shape)

        cv2.imshow('data_after_exp', data_after_exp)
        cv2.waitKey(5000)

    print('get_serial_number =', asi.get_serial_number(camera_id))

    asi.close_camera(camera_id)
except Exception as e:
    print(e)
