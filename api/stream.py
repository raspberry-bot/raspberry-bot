import tornado.ioloop
import tornado.web
from tornado.options import define, options
import shlex, subprocess, sys

DEFAULT_PORT = 9090

# TODO: The camera device should connect as a client. This app should 
# run a gstreamer server that can service multiple HTTP clients. Not 
# sure if it makes sense to support >1 camera device per instance of
# this application, might be interesting if the need arises.
DEFAULT_CAMERA_PORT = 4000

MP4_MUX_COMMAND = "gst-launch -q "                                          \
    "tcpclientsrc host={server} port={port} protocol=1 ! "                  \
    "'application/x-rtp, media=(string)video, clock-rate=(int)90000, "      \
        "encoding-name=(string)H264' ! "                                    \
    "rtph264depay ! queue ! "                                               \
    "h264parse ! queue ! "                                                  \
    "mp4mux streamable=true fragment-duration=5 ! "                         \
        "queue ! "                                                          \
    "filesink location=/dev/stderr"

# This causes the server to transcode to VP8. It's super slow on 1 CPU!
# Is there any way to parallelize this?
WEBM_MUX_COMMAND = "gst-launch -q "                                         \
    "tcpclientsrc host={server} port={port} protocol=1 ! "                  \
    "'application/x-rtp, media=(string)video, clock-rate=(int)90000, "      \
        "encoding-name=(string)H264' ! "                                    \
        "rtph264depay ! queue ! "                                           \
        "ffdec_h264 ! queue ! "                                             \
        "ffmpegcolorspace ! queue ! "                                       \
        "vp8enc ! queue ! "                                                 \
        "webmmux streamable=true ! queue ! "                                \
        "filesink  location=/dev/stderr"

# Right now only serving embedded H.264 video. WebM/VP8 is too slow (see
# comment above).

define("port", default=DEFAULT_PORT, help="run on the given port", type=int)
define("camera_ip", help="IP address of device serving camera stream", type=str)
define("camera_port", default=DEFAULT_CAMERA_PORT, help="Port of device serving camera stream", type=str)


class VideoStreamHandler(tornado.web.RequestHandler):
    _chunk_size = 65536

    def initialize(self, mux_command, video_format):
        self.video_format = video_format
        self.mux_command = mux_command.format(server=options.camera_ip, port=options.camera_port)

    @tornado.web.asynchronous
    def get(self):
        self.set_header("Content-Type", "video/{video_format}".format(video_format=self.video_format))
        self.muxer_process = subprocess.Popen(shlex.split(self.mux_command), stderr=subprocess.PIPE, bufsize=-1, close_fds=True)
        stream_fd = self.muxer_process.stderr.fileno()

        self.stream = tornado.iostream.PipeIOStream(stream_fd)
        self.stream.read_bytes(VideoStreamHandler._chunk_size, self.video_chunk)
        self.flush()

    def video_chunk(self, data):
        self.write(data)
        self.flush()
        self.stream.read_bytes(VideoStreamHandler._chunk_size, self.video_chunk)

    def cleanup_muxer(self):
        self.muxer_process.terminate()
        self.muxer_process.wait()

    def on_connection_close(self):
        self.stream.close()
        self.cleanup_muxer()
        self.finish()

def parse_cli_options():
    options.parse_command_line()
    if options.camera_ip is None:
        sys.stderr.write("ERROR: camera_ip is required\n\n")
        options.print_help()
        sys.exit(1)

application = tornado.web.Application([
    (r"/stream.mp4", VideoStreamHandler, dict(mux_command=MP4_MUX_COMMAND, video_format="mp4")),
    (r"/stream.webm", VideoStreamHandler, dict(mux_command=WEBM_MUX_COMMAND, video_format="webm")),
])

if __name__ == "__main__":
    parse_cli_options()
    application.listen(options.port)
    tornado.ioloop.IOLoop.instance().start()