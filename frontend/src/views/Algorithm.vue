<script setup>
import { ref, onMounted, onUnmounted } from "vue";

const videoUrl = ref("");

onMounted(() => {
  videoUrl.value = "/api/admin/video_feed";
});

// 组件卸载时关闭连接
onUnmounted(() => {
  videoUrl.value = "";
});
</script>

<template>
  <div class="algorithm-preview">
    <div class="video-container">
      <h2 class="video-title">画面实时预览</h2>
      <img :src="videoUrl" alt="实时视频流" class="video-stream" />
    </div>
  </div>
</template>

<style scoped>
.algorithm-preview {
  display: flex;
  justify-content: center;
  align-items: center;
  width: 84vw;
  height: 84vh;
  background: url("/src/assets/login_bg.svg") no-repeat center center;
  background-size: cover;
}

.video-container {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  width: 80%;
  height: 80%;
  padding: 10px;
  border-radius: 15px;
}

.video-title {
  color: black;
  font-size: 1.5rem;
  margin-bottom: 10px;
  font-weight: bold;
  text-align: center;
}

.video-stream {
  max-width: 100%;
  max-height: 100%;
  border-radius: 10px;
  object-fit: cover;
}
</style>
