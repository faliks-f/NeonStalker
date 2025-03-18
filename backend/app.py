import cv2
from av import VideoFrame
from flask_cors import CORS
from flask import Flask, request, jsonify, session, Response
import rclpy
from flask_jwt_extended import JWTManager
from threading import Thread

from routes.admin_controller import admin_bp
from routes.user_controller import user_bp
from ros_bridge.ros_bridge import ros_bridge


app = Flask(__name__)
CORS(app)

app.config["JWT_SECRET_KEY"] = "super-secret-key"  # 用于加密 JWT 令牌
app.config['SECRET_KEY'] = 'filesystem'
app.register_blueprint(user_bp)
app.register_blueprint(admin_bp)

jwt = JWTManager(app)

if __name__ == "__main__":
    # start a thread to run ros spin
    ros_thread = Thread(target=rclpy.spin, args=(ros_bridge,))
    ros_thread.start()

    app.run(host="0.0.0.0", port=9001, debug=True)
