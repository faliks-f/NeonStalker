import axios from "axios";
import router from "@/router"; // 引入 Vue Router

// 创建 Axios 实例
const instance = axios.create({
  baseURL: "", // 你的后端 API 地址
  timeout: 5000, // 超时时间
});

// 请求拦截器，自动携带 JWT 令牌
instance.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem("token");
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// 响应拦截器，处理 403 未登录状态
instance.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response && error.response.status === 403) {
      console.error("身份验证失败，登出用户");
      localStorage.removeItem("token");
      localStorage.removeItem("userRole");
      router.push("/api/login"); // 令牌失效，跳转到登录页面
    }
    return Promise.reject(error);
  }
);

export default instance;
