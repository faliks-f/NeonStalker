<template>
  <el-container class="main-layout">
    <!-- 侧边导航栏 -->
    <el-aside width="220px">
      <el-menu :default-active="activeMenu" class="el-menu-vertical" router>
        <el-menu-item index="/home">
<!--          <el-icon><House /></el-icon>-->
          <span>首页</span>
        </el-menu-item>
        <el-menu-item index="/live">
<!--          <el-icon><VideoCamera /></el-icon>-->
          <span>直播管理</span>
        </el-menu-item>
        <el-menu-item index="/settings">
<!--          <el-icon><Setting /></el-icon>-->
          <span>参数配置</span>
        </el-menu-item>
        <el-menu-item index="/algorithm">
<!--          <el-icon><Cpu /></el-icon>-->
          <span>算法预览</span>
        </el-menu-item>
        <el-menu-item index="/robot">
<!--          <el-icon><Cloud /></el-icon>-->
          <span>机器人状态</span>
        </el-menu-item>
        <el-menu-item index="/logs">
<!--          <el-icon><Document /></el-icon>-->
          <span>运行日志</span>
        </el-menu-item>
      </el-menu>
    </el-aside>

    <!-- 主体内容 -->
    <el-container>
      <el-header class="header">
        <span class="header-title">{{ currentPageTitle }}</span>

        <!-- 用户信息 -->
        <el-dropdown trigger="click">
          <span class="user-info">
            你好，{{ userRole }}
<!--            <el-icon><ArrowDown /></el-icon>-->
          </span>
          <template #dropdown>
            <el-dropdown-menu>
              <el-dropdown-item @click="handleLogout">退出登录</el-dropdown-item>
            </el-dropdown-menu>
          </template>
        </el-dropdown>
      </el-header>

      <el-main>
        <router-view />
      </el-main>
    </el-container>
  </el-container>
</template>

<script setup>
import { ref, computed } from "vue";
import { useRoute, useRouter } from "vue-router";
// import { House, VideoCamera, Setting, Cpu, Cloud, Document, ArrowDown } from "@element-plus/icons-vue";
import axios from "axios";

const route = useRoute();
const router = useRouter();
const activeMenu = ref(route.path);

// 获取当前页面标题
const currentPageTitle = computed(() => route.meta.title || "主页");

// 获取用户角色
const userRole = ref(localStorage.getItem("userRole") || "用户");

// 退出登录
const handleLogout = async () => {
  const logoutUrl = userRole.value === "管理员" ? "/api/admin/logout" : "/api/user/logout";
  const token = localStorage.getItem("token");
  await axios.post(logoutUrl, null, {
    headers: {
      Authorization: `Bearer ${token}`,
    },
  });
  localStorage.removeItem("userRole");
  router.push("/login");
};
</script>

<style scoped>
.main-layout {
  height: 100vh;
  display: flex;
}

/* 侧边导航栏 */
.el-aside {
  background-color: #f8f9fa;
  border-right: 1px solid #ddd;
}

/* 头部 */
.header {
  height: 60px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 20px;
  font-size: 18px;
  font-weight: bold;
  background-color: white;
  border-bottom: 1px solid #ddd;
}

/* 用户信息 */
.user-info {
  cursor: pointer;
  font-size: 16px;
  display: flex;
  align-items: center;
  gap: 5px;
}
</style>
