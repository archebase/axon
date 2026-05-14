import { defineConfig, loadEnv } from 'vite'
import vue from '@vitejs/plugin-vue'
import { resolve } from 'path'

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const recorderPort = env.AXON_RECORDER_PORT || env.VITE_RECORDER_PORT || 8080
  const panelPort = env.AXON_PANEL_API_PORT || env.VITE_PANEL_API_PORT || 8082

  return {
    plugins: [vue()],
    resolve: {
      alias: {
      '@': resolve(__dirname, './src')
      }
    },
    server: {
      host: '0.0.0.0',
      proxy: {
        '/rpc': {
          target: `http://localhost:${recorderPort}`,
          changeOrigin: true
        },
        '/health': {
          target: `http://localhost:${recorderPort}`,
          changeOrigin: true
        },
        '/api/panel': {
          target: `http://localhost:${panelPort}`,
          changeOrigin: true
        }
      }
    }
  }
})
