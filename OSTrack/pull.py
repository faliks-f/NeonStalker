import cv2
import subprocess
import numpy as np

# RTMP 或其他协议的视频流地址
video_stream_url = "rtmp://localhost:1935/test"  # 替换为实际的流地址
# FFmpeg 命令
ffmpeg_cmd = [
    "ffmpeg",
    "-i", video_stream_url,  # 输入流地址
    "-f", "rawvideo",  # 输出原始视频格式
    "-pix_fmt", "bgr24",  # OpenCV 使用的像素格式
    "-vcodec", "rawvideo",  # 原始视频编码
    "-an",  # 无音频
    "-sn",  # 无字幕
    "-"
]

# 启动 FFmpeg 进程
process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**8)

# 视频参数（根据实际流调整）
frame_width = 720  # 视频宽度
frame_height = 480  # 视频高度
frame_size = frame_width * frame_height * 3  # 单帧字节数 (BGR24)

print("拉取视频流并显示，按 'q' 键退出")

try:
    while True:
        # 从 FFmpeg 标准输出读取数据
        raw_frame = process.stdout.read(frame_size)

        if not raw_frame:
            print("视频流结束或中断")
            break

        # 将原始数据转换为 NumPy 数组，并重塑为视频帧
        frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((frame_height, frame_width, 3))

        # 在 OpenCV 窗口中显示视频
        cv2.imshow("Video Stream", frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except Exception as e:
    print(f"发生错误: {e}")
finally:
    # 释放资源
    process.terminate()
    cv2.destroyAllWindows()