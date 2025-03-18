<script setup>
import { ref } from "vue";

// 绑定滑动条的值（默认 2000mm）
const followDistance = ref(2000);

// 点击高度按钮的逻辑
const setCameraHeight = (height) => {
  console.log(`设置镜头高度: ${height}`);
};
</script>

<template>
  <div class="settings-container">
    <!-- 左侧参数配置 -->
    <div class="settings-panel">
      <h2>镜头跟拍距离</h2>
      <div class="slider-value">{{ followDistance }}mm</div>
      <div class="slider-container">
        <span>1000</span>
        <input type="range" v-model="followDistance" min="1000" max="4000" class="slider" />
        <span>4000</span>
      </div>

      <h2>镜头高度</h2>
      <div class="button-group">
        <button @click="setCameraHeight('high')">抬升</button>
        <button @click="setCameraHeight('medium')">复位</button>
        <button @click="setCameraHeight('low')">降低</button>
      </div>
    </div>

    <!-- 右侧机器人图片 -->
    <div class="robot-image">
      <img src="@/assets/robot.png" alt="机器人" />
    </div>
  </div>
</template>

<style scoped>
.settings-container {
  display: flex;
  width: 100%;
  height: 100vh;
  background: url("@/assets/login_bg.svg") no-repeat center;
  background-size: cover;
}

.settings-panel {
  width: 30%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  gap: 15px;
}

.settings-panel h2 {
  font-size: 20px;
  color: #333;
}

/* 滑动条上方数值显示 */
.slider-value {
  font-size: 18px;
  font-weight: bold;
  color: #000000;
}

.slider-container {
  display: flex;
  align-items: center;
  width: 66%; /* 2/3 长度 */
  justify-content: space-between;
}

.slider {
  flex-grow: 1;
  margin: 0 10px;
}

.button-group {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

button {
  width: 100px;
  padding: 10px;
  font-size: 16px;
  border: none;
  cursor: pointer;
  background-color: #007bff;
  color: white;
  border-radius: 5px;
}

button:hover {
  background-color: #0056b3;
}

.robot-image {
  width: 70%;
  display: flex;
  justify-content: center;
  align-items: center;
}

.robot-image img {
  max-width: 100%;
  height: auto;
}
</style>
