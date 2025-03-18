<template>
  <div class="live-container">
    <!-- 按钮区域（顶部 10%） -->
    <div class="button-panel">
      <button @click="controlGripper('release')">张开加爪</button>
      <button @click="controlGripper('clamp')">松开加爪</button>
      <button @click="startStream">直播开始</button>
      <button @click="stopStream">直播结束</button>
    </div>

    <!-- 图片展示区域（下方 90%） -->
    <div class="image-panel">
      <img src="@/assets/login_left3.png" alt="直播管理">
    </div>
  </div>
</template>

<script>
import axios from "axios";

export default {
  methods: {
    // 控制加爪动作
    async controlGripper(action) {
      try {
        const token = localStorage.getItem("token");
        const response = await axios.post("/api/user/gripper_control", {
          action: action,
          headers: {
            Authorization: `Bearer ${token}`,
          },
        });
        console.log("加爪控制成功:", response.data);
      } catch (error) {
        console.error("加爪控制失败:", error.response ? error.response.data : error.message);
      }
    },

    // 直播开始
    async startStream() {
      try {
        const response = await axios.post("/api/user/start_stream");
        console.log("直播已开始:", response.data);
      } catch (error) {
        console.error("直播开始失败:", error.response ? error.response.data : error.message);
      }
    },

    // 直播结束
    async stopStream() {
      try {
        const response = await axios.post("/api/user/stop_stream");
        console.log("直播已结束:", response.data);
      } catch (error) {
        console.error("直播结束失败:", error.response ? error.response.data : error.message);
      }
    },
  },
};
</script>

<style scoped>
/* 页面布局 */
.live-container {
  display: flex;
  flex-direction: column;
  width: 100%;
  height: 70vh;
  //background: url("@/assets/login_bg.svg") no-repeat center center;
  background-size: cover;
}

/* 顶部按钮区域 */
.button-panel {
  height: 10%;
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 30px;
  background: rgba(255, 255, 255, 0.2);
  padding: 10px 0;
}

/* 按钮样式 */
.button-panel button {
  padding: 10px 20px;
  font-size: 16px;
  background-color: #007bff;
  color: white;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  transition: 0.3s;
}

.button-panel button:hover {
  background-color: #0056b3;
}

/* 图片区域 */
.image-panel {
  height: 90%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.image-panel img {
  max-width: 40%;
  max-height: 60%;
  object-fit: contain;
}
</style>
