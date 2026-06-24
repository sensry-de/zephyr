//
// Created by tswaehn on 7/15/26.
//

#ifndef USB_VIDEO_HTPA_SENS_H
#define USB_VIDEO_HTPA_SENS_H


#include <zephyr/device.h>
#include <zephyr/drivers/video.h>

int htpa_init_sensor(const struct device *dev);
int htpa_copy_frame(const struct device *dev, struct video_buffer *vbuf);

#endif //USB_VIDEO_HTPA_SENS_H
