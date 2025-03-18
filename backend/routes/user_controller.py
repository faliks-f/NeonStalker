from flask import Blueprint, request, jsonify, session
from service.user_service import UserService
from flask_jwt_extended import (
    JWTManager, create_access_token, jwt_required, get_jwt_identity
)
from service.log_service import log_service
user_bp = Blueprint('user', __name__)
user_service = UserService()

def require_login():
    """通用的登录状态检查"""
    pass
    # if not session.get("logged_in"):
    #     return jsonify({"error": "User not logged in"}), 403
    # identity = get_jwt_identity()
    # if identity["user_role"] != "user":
    #     return jsonify({"msg": "Access forbidden"}), 403


@user_bp.route("/user/login", methods=["POST"])
def login():
    """用户登录"""
    data = request.json
    username = data.get("username")
    password = data.get("password")

    if not username or not password:
        return jsonify({"error": "Missing username or password"}), 400

    # 检查是否已有用户登录
    # if session.get("logged_in"):
    #     return jsonify({"error": "User already logged in"}), 403

    success, message = user_service.validate_user(username, password)
    if success:
        # session["logged_in"] = True  # 标记用户已登录
        access_token = create_access_token(identity={"username": "user", "role": "user"})
        log_service.write_log(
            role="user",
            action="登录成功",
            level="INFO",
            module="All"
        )

        return jsonify({"message": message, "access_token": access_token, "success": True}), 200
    else:
        log_service.write_log(
            role="user",
            action="登录失败",
            level="WARNING",
            module="All"
        )
        return jsonify({"error": message}), 403

@user_bp.route("/user/logout", methods=["POST"])
# @jwt_required()
def logout():
    """用户登出"""
    login_check = require_login()
    if login_check:
        return login_check
    log_service.write_log(
        role="user",
        action="登出",
        level="INFO",
        module="All"
    )
    session.pop("logged_in", None)  # 清除登录状态
    return jsonify({"message": "User logged out successfully"}), 200

@user_bp.route("/user/start_stream", methods=["POST"])
# @jwt_required()
def start_stream():
    """开始直播"""
    login_check = require_login()
    if login_check:
        return login_check
    log_service.write_log(
        role="user",
        action="开始直播",
        level="INFO",
        module="All"
    )
    user_service.start_stream()
    return jsonify({"message": "Stream started successfully"}), 200

@user_bp.route("/user/stop_stream", methods=["POST"])
# @jwt_required()
def stop_stream():
    """停止直播"""

    login_check = require_login()
    if login_check:
        return login_check
    log_service.write_log(
        role="user",
        action="停止直播",
        level="INFO",
        module="All"
    )
    user_service.stop_stream()
    return jsonify({"message": "Stream stopped successfully"}), 200

@user_bp.route("/user/set_track_distance", methods=["POST"])
# @jwt_required()
def set_track_distance():
    """设置目标跟踪距离"""
    login_check = require_login()
    if login_check:
        return login_check


    log_service.write_log(
        role="user",
        action="设置目标跟踪距离",
        level="INFO",
        module="Car"
    )

    data = request.json
    distance = data.get("distance")

    if distance is None or not isinstance(distance, (int, float)) or distance <= 0:
        return jsonify({"error": "Invalid distance"}), 400
    user_service.set_track_distance(distance)

    return jsonify({"message": f"Tracking distance set to {distance}"}), 200

@user_bp.route("/user/set_camera_height", methods=["POST"])
# @jwt_required()
def set_camera_height():
    """设置摄像头高度"""
    login_check = require_login()
    if login_check:
        return login_check
    log_service.write_log(
        role="user",
        action="设置摄像头高度",
        level="INFO",
        module="Arm"
    )
    data = request.json
    height = data.get("height")
    user_service.set_camera_height(height)
    if height is None or not isinstance(height, (int, float)) or height <= 0:
        return jsonify({"error": "Invalid height"}), 400


    return jsonify({"message": f"Camera height set to {height}"}), 200

@user_bp.route("/user/gripper_control", methods=["POST"])
# @jwt_required()
def gripper_control():
    login_check = require_login()
    if login_check:
        return login_check

    log_service.write_log(
        role="user",
        action=f"控制夹爪",
        level="INFO",
        module="Arm"
    )
    data = request.json
    action = data.get("action")
    res = user_service.gripper_control(action)
    if res:
        return res
    return jsonify({"message": f"Gripper {action}"}), 200