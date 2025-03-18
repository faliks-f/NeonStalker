import { createRouter, createWebHistory } from "vue-router";
// import MainLayout from "@/layout/MainLayout.vue";
import Login from "@/views/Login.vue";
import MainLayout from "@/layout/MainLayout.vue";
import home from "@/views/Home.vue";
import Home from "@/views/Home.vue";
import Live from "@/views/Live.vue";
import Settings from "@/views/Settings.vue";
import Algorithm from "@/views/Algorithm.vue";
import Robot from "@/views/Robot.vue";
import Logs from "@/views/Logs.vue";

const routes = [
  { path: "/login", component: Login },
  {
    path: "/",
    component: MainLayout,
    children: [
      {path: "", redirect: "home"},
      { path: "home", component: Home, meta: { title: "首页" } },
      { path: "live", component: Live, meta: { title: "直播管理" } },
      { path: "settings", component: Settings, meta: { title: "参数配置" } },
      { path: "algorithm", component: Algorithm, meta: { title: "算法预览" } },
      { path: "robot", component: Robot, meta: { title: "机器人状态" } },
      { path: "logs", component: Logs, meta: { title: "运行日志" } },
    ],
  }
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

// 登录拦截
router.beforeEach((to, from, next) => {
  const isAuthenticated = localStorage.getItem("userRole"); // 通过 localStorage 判断登录状态
  if (to.path !== "/login" && !isAuthenticated) {
    next("/login");
  } else {
    next();
  }
});

export default router;
