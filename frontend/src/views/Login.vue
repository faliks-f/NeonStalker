<template>
  <div class="login-container">
    <!-- 背景图片 -->
    <div class="background"></div>

    <!-- 登录面板 -->
    <el-card class="login-card">
      <!-- 装饰图案 -->
      <img src="@/assets/login_bg.svg" class="login-bg" />

      <!-- 标题 -->
      <h2 class="login-title">Geeker-Admin</h2>

      <!-- 登录表单 -->
      <el-form :model="form" class="login-form">
        <!-- 身份选择下拉框 -->
        <el-form-item>
          <el-select v-model="form.identity" placeholder="请选择身份">
            <el-option label="公共帐号" value="user" />
            <el-option label="管理员帐号" value="admin" />
          </el-select>
        </el-form-item>

        <!-- 用户名输入框 -->
        <el-form-item>
          <el-input v-model="form.username" placeholder="用户名" />
        </el-form-item>

        <!-- 密码输入框 -->
        <el-form-item>
          <el-input v-model="form.password" type="password" placeholder="密码" show-password />
        </el-form-item>

        <!-- 登录失败提示 -->
        <p v-if="errorMessage" class="error-message">{{ errorMessage }}</p>

        <!-- 按钮区域 -->
        <el-form-item>
          <el-button type="primary" class="login-button" @click="login">登录</el-button>
        </el-form-item>
        <el-form-item>
          <el-button type="info" class="reset-button" @click="resetForm">重置</el-button>
        </el-form-item>
      </el-form>
    </el-card>
  </div>
</template>

<script>
import { ref } from "vue";
import { useRouter } from "vue-router";
import axios from "axios";
import { ElMessage } from "element-plus";

export default {
  setup() {
    const router = useRouter();
    const errorMessage = ref("");

    const form = ref({
      identity: "user", // 默认选择公共帐号
      username: "",
      password: "",
    });

    const role = ref("用户");

    const login = async () => {
      const loginUrl = role.value === "管理员" ? "/api/admin/login" : "/api/user/login";
      try {
        const response = await axios.post(loginUrl, {
          username: form.value.username,
          password: form.value.password,
        });

        if (response.data.success) {
          // ElMessage.success("登录成功！");
          localStorage.setItem("userRole", role.value);
          localStorage.setItem("token", response.data.access_token); // 存储 JWT 令牌
          console.log("当前用户角色:", localStorage.getItem("userRole")); // 确保成功写入
          router.push("/"); // 跳转到主页面
        } else {
          errorMessage.value = "帐号或密码错误";
        }
      } catch (error) {
        errorMessage.value = "帐号或密码错误";
      }
    };

    const resetForm = () => {
      form.value.identity = "user";
      form.value.username = "";
      form.value.password = "";
      errorMessage.value = "";
    };

    return { form, login, resetForm, errorMessage };
  },
};
</script>

<style scoped>
/* 背景图片 */
.background {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: url("@/assets/img.png") no-repeat center center;
  background-size: cover;
  z-index: -1;
}

/* 登录页面居中 */
.login-container {
  display: flex;
  justify-content: center;
  align-items: center;
  height: 100vh;
}

/* 登录框 */
.login-card {
  width: 350px;
  padding: 20px;
  text-align: center;
  position: relative;
  background: rgba(255, 255, 255, 0.9);
  border-radius: 10px;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
}

/* 背景装饰图 */
.login-bg {
  width: 80px;
  position: absolute;
  top: -40px;
  left: 50%;
  transform: translateX(-50%);
}

/* 标题 */
.login-title {
  margin-bottom: 20px;
  font-size: 22px;
  font-weight: bold;
}

/* 登录按钮 */
.login-button {
  width: 100%;
}

/* 重置按钮 */
.reset-button {
  width: 100%;
}

/* 登录失败提示 */
.error-message {
  color: red;
  font-size: 14px;
  text-align: center;
  margin-bottom: 10px;
}
</style>
