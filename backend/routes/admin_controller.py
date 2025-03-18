from datetime import datetime

from aiortc import RTCPeerConnection
from aiortc.contrib.media import MediaRelay
from flask import Blueprint, request, jsonify, session, render_template, Response
from flask_jwt_extended import (
    JWTManager, create_access_token, jwt_required, get_jwt_identity
)

from service.log_service import log_service
from service.admin_service import AdminService

admin_bp = Blueprint('admin', __name__)
admin_service = AdminService()

@admin_bp.route("/admin/login", methods=["POST"])
def admin_login():
    """管理员登录"""
    data = request.json
    username = data.get("username")
    password = data.get("password")



    if not username or not password:
        return jsonify({"error": "Missing username or password"}), 400

    # 检查是否已有用户登录
    # if session.get("logged_in"):
    #     return jsonify({"error": "User already logged in"}), 403

    success, message = admin_service.validate_user(username, password)


    if success:
        log_service.write_log(
            role="admin",
            action="登录成功",
            level="INFO",
            module="All"
        )

        session["logged_in"] = True  # 标记用户已登录
        access_token = create_access_token(identity={"username": "admin", "role": "admin"})
        return jsonify({"message": message, "success": True, "access_token": access_token}), 200
    else:
        log_service.write_log(
            role="admin",
            action="登录失败",
            level="WARNING",
            module="All"
        )

        return jsonify({"error": message}), 403

@admin_bp.route('/admin/video_feed')
def video_feed():
    """视频流路由"""
    return Response(
        admin_service.generate_mjpeg(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@admin_bp.route("/admin/arm_joint", methods=["POST"])
def get_arm_joint():
    return jsonify(admin_service.get_arm_joint())

@admin_bp.route("/admin/arm_pose", methods=["POST"])
def get_arm_pose():
    return jsonify(admin_service.get_arm_pose())

@admin_bp.route("/admin/gripper_dis", methods=["POST"])
def get_arm_gripper_dis():
    return jsonify(admin_service.get_arm_gripper_dis())

@admin_bp.route("/admin/car_speed", methods=["POST"])
def get_car_speed():
    return jsonify(admin_service.get_car_speed())

@admin_bp.route("/admin/laser_dis", methods=["POST"])
def get_laser_dis():
    return jsonify(admin_service.get_laser_dis())


@app.route("/api/logs/query", methods=["POST"])
def query_logs():
    """
    查询日志
    前端发送 JSON 数据：
    {
        "role": "user",           # 可选
        "action": "开始直播",      # 可选
        "level": "INFO",          # 可选
        "module": "Camera",       # 可选
        "start_date": "2025-03-01", # 可选
        "end_date": "2025-03-18"    # 可选
    }
    """
    data = request.get_json()

    # 初始化查询
    query = log_service.query()

    # 过滤条件（支持链式调用）
    if "role" in data:
        query = query.filter_by_role(data["role"])
    if "action" in data:
        query = query.filter_by_action(data["action"])
    if "level" in data:
        query = query.filter_by_level(data["level"])
    if "module" in data:
        query = query.filter_by_module(data["module"])
    if "start_date" in data and "end_date" in data:
        try:
            start_date = datetime.strptime(data["start_date"], "%Y-%m-%d")
            end_date = datetime.strptime(data["end_date"], "%Y-%m-%d")
            query = query.filter_by_date_range(start_date, end_date)
        except ValueError:
            return jsonify({"error": "日期格式错误"}), 400

    # 执行查询
    logs = query.execute()
    return jsonify({"logs": logs}), 200