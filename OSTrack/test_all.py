import importlib
import time

import cv2

from real_sense import RealsenseCamera
from ultralytics import YOLO

real_sense = RealsenseCamera()


def _build_init_info(box):
    return {'init_bbox': box}


def test_align():
    color_frame, depth_image, depth_intrin, depth_frame = real_sense.get_aligned_images_for_test()


def test_yolo():
    yolo_model = YOLO("yolo11m.pt")
    color_frame, depth_image, depth_intrin, depth_frame = real_sense.get_aligned_images()
    color_frame = cv2.flip(color_frame, 1)
    depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
    time1 = time.time()
    results = yolo_model(color_frame,
                         classes=[0],
                         conf=0.3,
                         iou=0.5,
                         )
    time2 = time.time()
    print("yolo time: ", time2 - time1)


def test_track():
    color_frame, depth_image, depth_intrin, depth_frame = real_sense.get_aligned_images()
    color_frame = cv2.flip(color_frame, 1)
    param_module = importlib.import_module('lib.test.parameter.{}'.format("ostrack"))
    params = param_module.parameters('vitb_384_mae_ce_32x4_ep300')
    tracker_module = importlib.import_module('lib.test.tracker.{}'.format("ostrack"))
    tracker = tracker_module.get_tracker_class()(params, "otb")
    time1 = time.time()
    tracker.initialize(color_frame, _build_init_info([100, 200, 300, 400]))
    time2 = time.time()
    print("ostrack init time: ", time2 - time1)
    out = tracker.track(color_frame)
    time3 = time.time()
    print("ostrack track time: ", time3 - time2)



if __name__ == '__main__':
    test_align()
    test_yolo()
    test_track()
