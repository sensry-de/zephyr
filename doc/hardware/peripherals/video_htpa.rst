.. _video_htpa:

HTPA thermal imaging pipeline
#############################

The Heimann HTPA video drivers support 120 x 84 and 160 x 120 thermal array
sensors. The pipeline consists of three video devices:

.. code-block:: text

   +--------+     +-------------+     +-------------+     +-----------+     +-------------+
   | Sensor | --> | Acquisition | --> | Compensator | --> | Colorizer | --> | Application |
   +--------+     +-------------+     +-------------+     +-----------+     +-------------+
                         RAW                 S16              S16, GREY8
                                                              or RGB565

* The acquisition device reads raw frames over SPI, including periodically
  acquired electrical offsets. Its output format is ``VIDEO_PIX_FMT_HTPA_RAW``.
* The compensator corrects electrical offsets and sorts pixels into signed
  16-bit samples in ``VIDEO_PIX_FMT_HTPA_S16`` format.
* The colorizer passes through S16 samples or converts them to GREY8 or RGB565.
  RGB565 uses the Ironbow, Rainbow, or Inferno palette.

Each processing device references its upstream device through its devicetree
``source`` property. For example, given an enabled sensor node labeled
``htpa_sensor``::

   htpa_compensator: htpa-compensator {
           compatible = "heimann,htpa-compensator";
           source = <&htpa_sensor>;
   };

   htpa_colorizer: htpa-colorizer {
           compatible = "heimann,htpa-colorizer";
           source = <&htpa_compensator>;
   };

Enable the acquisition, compensator, and colorizer drivers for the complete
pipeline. Each stage requires exclusive ownership of its upstream device.
Applications can use the colorizer as the video source, for example with the
USB UVC sample.

The colorizer exposes ``VIDEO_CID_HTPA_PALETTE`` for runtime palette selection.
The control and palette values are declared in
``zephyr/drivers/video/htpa_colorizer.h``. Ironbow is the default.

Buffer ownership
****************

The application allocates streaming buffers and enqueues them on the final
video device. Successful enqueue transfers ownership to the device; dequeue
returns it to the application, which can consume and requeue the buffer.
Multiple buffers can be in flight, allowing acquisition and processing to overlap.

The format's ``size`` is the buffer capacity required for the complete pipeline;
``bytesused`` gives the output payload size. Buffer count and alignment
requirements reflect the upstream device, and processing preserves timestamps.

