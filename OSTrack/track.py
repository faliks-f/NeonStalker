import threading
import importlib
import cv2
import rclpy
from std_msgs.msg import Int8, MultiArrayDimension, Int16MultiArray

from ultralytics import YOLO

from rclpy.node import Node
from real_sense import RealsenseCamera

import subprocess


# rtmp_url= "rtmp://localhost:1935/live/test"


def _build_init_info(box):
    return {'init_bbox': box}


class Track(Node):
    def __init__(self):
        super().__init__('track')
        # self.tracker = Tracker('ostrack', 'vitb_384_mae_ce_32x4_ep300', 'otb')
        param_module = importlib.import_module('lib.test.parameter.{}'.format("ostrack"))
        params = param_module.parameters('vitb_384_mae_ce_32x4_ep300')
        tracker_module = importlib.import_module('lib.test.tracker.{}'.format("ostrack"))
        self.tracker = tracker_module.get_tracker_class()(params, "otb")

        self.yolo_model = YOLO("yolo11m.pt")
        self.real_sense = RealsenseCamera()

        self.track_result_publisher = self.create_publisher(Int16MultiArray, 'track_result', 10)
        self.start_track_subscriber = self.create_subscription(Int8, '/total_cmd', self.start_track_callback, 10)

        self.start_track = False
        self.init_track = False

        self.message = None

        # ffmpeg_cmd = [
        #     "ffmpeg",
        #     "-y",  # 覆盖输出文件
        #     "-f", "rawvideo",  # 原始视频流格式
        #     "-vcodec", "rawvideo",  # 原始视频编码
        #     "-pix_fmt", "bgr24",  # OpenCV 使用 BGR 格式
        #     "-s", f"{640}x{480}",  # 视频分辨率
        #     "-r", str(30),  # 帧率
        #     "-i", "-",  # 输入来自标准输入
        #     "-c:v", "h264_nvenc",  # 使用 x264 编码器
        #     "-pix_fmt", "yuv420p",  # 像素格式
        #     "-preset", "llhq",  # 编码速度
        #     "-f", "flv",  # 输出格式为 FLV
        #     rtmp_url  # 推流地址
        # ]
        # self.ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

        thread = threading.Thread(target=self.run_video, args=(True,))
        thread.start()

    def yolo_process(self, debug=False):
        color_frame, depth_image, depth_intrin, depth_frame = self.real_sense.get_aligned_images()
        color_frame = cv2.flip(color_frame, 1)
        # self.ffmpeg_process.stdin.write(color_frame.tobytes())
        results = self.yolo_model(color_frame,
                                  classes=[0],
                                  conf=0.3,
                                  iou=0.5,
                                  )
        detections = results[0].boxes.data.cpu().numpy()
        if len(detections) == 0:
            return color_frame, None
        x_min, y_min, x_max, y_max = map(int, detections[0][:4])
        max_res = [x_min, y_min, x_max - x_min, y_max - y_min]
        for det in detections:
            x_min, y_min, x_max, y_max = map(int, det[:4])
            w, h = x_max - x_min, y_max - y_min
            if (w * h) > (max_res[2] * max_res[3]):
                max_res = [x_min, y_min, w, h]
            if debug:
                conf, cls = float(det[4]), int(det[5])
                label = f"{self.yolo_model.names[cls]} {conf:.2f}"
                # 绘制检测框
                cv2.rectangle(color_frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 3)
                # 显示类别和置信度
                cv2.putText(color_frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return color_frame, max_res

    def run_video(self, debug=False):
        while rclpy.ok():
            if not self.start_track and not self.init_track:
                color_frame, max_res = self.yolo_process(debug)
                # self.ffmpeg_process.stdin.write(color_frame.tobytes())
                if debug:
                    cv2.imshow("track", color_frame)
                    cv2.waitKey(1)
            elif not self.init_track:
                color_frame, max_res = self.yolo_process(debug)
                # self.ffmpeg_process.stdin.write(color_frame.tobytes())
                if debug:
                    cv2.imshow("track", color_frame)
                    cv2.waitKey(1)
                if max_res is not None:
                    self.tracker.initialize(color_frame, _build_init_info(max_res))
                    self.init_track = True
            else:
                color_frame, depth_image, depth_intrin, depth_frame = self.real_sense.get_aligned_images()
                # self.ffmpeg_process.stdin.write(color_frame.tobytes())
                color_frame = cv2.flip(color_frame, 1)
                out = self.tracker.track(color_frame)
                state = [int(s) for s in out['target_bbox']]
                if len(state) == 0:
                    continue
                if debug:
                    cv2.rectangle(color_frame, (state[0], state[1]), (state[2] + state[0], state[3] + state[1]),
                                  (0, 0, 255), 5)
                    cv2.circle(color_frame, (state[0] + state[2] // 2, state[1] + state[3] * 2 // 3), 5,
                               (0, 0, 255), -1)
                    cv2.imshow("track", color_frame)
                    cv2.waitKey(1)

                if self.message is None:
                    self.message = Int16MultiArray()
                    self.message.layout.dim = [MultiArrayDimension()]
                    self.message.layout.dim[0].label = "detections"
                    self.message.layout.dim[0].size = 1
                    self.message.layout.dim[0].stride = 5

                distance = self.real_sense.get_coordinate_3d(state[0] + state[2] // 2, state[1] + state[3] * 2 // 3,
                                                             depth_frame)
                flattened_data = [state[0], state[1], state[2] + state[0], state[3] + state[1], int(distance * 1000)]
                self.message.data = flattened_data
                self.track_result_publisher.publish(self.message)

    def start_track_callback(self, msg):
        if msg.data == 3:
            self.start_track = True
        elif msg.data == 9:
            self.start_track = False
            self.init_track = False


def main(args=None):
    rclpy.init(args=args)
    track = Track()
    rclpy.spin(track)


if __name__ == '__main__':
    main()
