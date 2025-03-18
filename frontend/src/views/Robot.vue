<template>
  <div class="robot-status">
    <!-- 机械臂参数 -->
    <div class="section">
      <h3>机械臂状态</h3>

      <!-- 关节角状态 -->
      <h4>关节角状态</h4>
      <div class="grid-container">
        <div class="data-box" v-for="(angle, index) in jointAngles" :key="index">
          <span class="label">关节{{ index + 1 }}:</span>
          <div class="value-box">{{ angle }}</div>
          <span class="unit">°</span>
        </div>
      </div>

      <!-- 空间点位状态 -->
      <h4>空间点位状态</h4>
      <div class="grid-container">
        <div class="data-box"><span class="label">X:</span> <div class="value-box">{{ pose.x }}</div> <span class="unit">mm</span></div>
        <div class="data-box"><span class="label">Y:</span> <div class="value-box">{{ pose.y }}</div> <span class="unit">mm</span></div>
        <div class="data-box"><span class="label">Z:</span> <div class="value-box">{{ pose.z }}</div> <span class="unit">mm</span></div>
        <div class="data-box"><span class="label">Rx:</span> <div class="value-box">{{ pose.rx }}</div> <span class="unit">°</span></div>
        <div class="data-box"><span class="label">Ry:</span> <div class="value-box">{{ pose.ry }}</div> <span class="unit">°</span></div>
        <div class="data-box"><span class="label">Rz:</span> <div class="value-box">{{ pose.rz }}</div> <span class="unit">°</span></div>
      </div>

      <!-- 夹爪状态 -->
      <h4>夹爪状态</h4>
      <div class="data-box">
        <span class="label">夹爪张开距离:</span>
        <div class="value-box">{{ gripperDistance }}</div>
        <span class="unit">mm</span>
      </div>
    </div>

    <!-- 运动底盘参数 -->
    <div class="section">
      <h3>底盘状态</h3>
      <div class="grid-container">
        <div class="data-box"><span class="label">X 轴线速度:</span> <div class="value-box">{{ velocity.x }}</div> <span class="unit">m/s</span></div>
        <div class="data-box"><span class="label">Y 轴线速度:</span> <div class="value-box">{{ velocity.y }}</div> <span class="unit">m/s</span></div>
        <div class="data-box"><span class="label">Z 轴角速度:</span> <div class="value-box">{{ velocity.z }}</div> <span class="unit">rad/s</span></div>
      </div>
    </div>

    <!-- 激光雷达数据 -->
    <div class="section">
      <h3>激光雷达</h3>
      <div class="data-box">
        <span class="label">测距距离:</span>
        <div class="value-box">{{ lidarDistance }}</div>
        <span class="unit">mm</span>
      </div>
      <div class="obstacle-indicator">
        <span :class="hasObstacle ? 'red-circle' : 'green-circle'"></span>
        <span>{{ hasObstacle ? "前方有障碍物" : "前方无障碍物" }}</span>
      </div>
    </div>
  </div>
</template>

<script>
import axios from "axios";

export default {
  data() {
    return {
      jointAngles: [0, 0, 0, 0, 0, 0], // 机械臂六轴角度
      pose: { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }, // 机械臂位姿
      gripperDistance: 0, // 夹爪张开距离
      velocity: { x: 0, y: 0, z: 0 }, // 运动底盘速度
      lidarDistance: 0, // 激光雷达测距
      hasObstacle: false, // 是否有障碍物
    };
  },
  mounted() {
    this.fetchData();
    this.interval = setInterval(this.fetchData, 1000); // 每秒更新一次数据
  },
  beforeUnmount() {
    clearInterval(this.interval);
  },
  methods: {
    async fetchData() {
      try {
        const [jointRes, poseRes, gripperRes, speedRes, lidarRes] = await Promise.all([
          axios.post("/api/admin/arm_joint"),
          axios.post("/api/admin/arm_pose"),
          axios.post("/api/admin/gripper_dis"),
          axios.post("/api/admin/car_speed"),
          axios.post("/api/admin/laser_dis"),
        ]);

        this.jointAngles = jointRes.data || this.jointAngles;
        this.pose = poseRes.data || this.pose;
        this.gripperDistance = gripperRes.data || this.gripperDistance;
        this.velocity = speedRes.data || this.velocity;
        this.lidarDistance = lidarRes.data || this.lidarDistance;
        this.hasObstacle = this.lidarDistance < 60;
      } catch (error) {
        console.error("获取机器人状态失败", error);
      }
    },
  },
};
</script>

<style scoped>
.robot-status {
  display: flex;
  flex-direction: column;
  gap: 15px;
  padding: 20px;
  background: url("@/assets/login_bg.svg") no-repeat center center;
  background-size: cover;
}

.section {
  padding: 10px;
  background: rgba(255, 255, 255, 0.8);
  border-radius: 8px;
}

h3 {
  margin-bottom: 10px;
}

h4 {
  margin-top: 10px;
  font-size: 16px;
  font-weight: bold;
}

/* 控制数据排列，减少间距 */
.grid-container {
  display: grid;
  grid-template-columns: repeat(2, minmax(140px, 1fr));
  gap: 8px;
}

/* 让文本和框对齐 */
.data-box {
  display: flex;
  align-items: center;
  gap: 6px;
}

/* 文字样式 */
.label {
  min-width: 100px;
  text-align: right;
  font-weight: bold;
  line-height: 32px; /* 让文字与框等高 */
}

/* 白底圆角数据框 */
.value-box {
  display: flex;
  justify-content: center;
  align-items: center;
  min-width: 60px;
  height: 32px;
  background: #fff;
  border: 1px solid #ccc;
  border-radius: 6px;
  text-align: center;
  font-weight: bold;
  color: #333;
}

/* 单位 */
.unit {
  font-size: 14px;
  color: #666;
}

.obstacle-indicator {
  display: flex;
  align-items: center;
  gap: 10px;
}

.red-circle, .green-circle {
  width: 18px;
  height: 18px;
  border-radius: 50%;
}

.red-circle {
  background-color: red;
}

.green-circle {
  background-color: green;
}
</style>
