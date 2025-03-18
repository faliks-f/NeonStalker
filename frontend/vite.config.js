import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    vue(),
    vueDevTools(),
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url))
    },

  },
  server: {  // 新增server配置
    proxy: {
      // 代理所有以/api开头的请求
      '/api': {
        target: 'http://localhost:9001',  // 这里改成你的实际后端地址
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/api/, '') // 可选：移除/api前缀
      }
      // 如果需要代理多个路径，可以继续添加配置
    }
  }
})

