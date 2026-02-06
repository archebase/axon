import axios from 'axios'

const BASE_URL = import.meta.env.VITE_API_BASE_URL || ''

// Create axios instance with interceptors for debugging
const apiClient = axios.create({
  baseURL: BASE_URL,
  headers: {
    'Content-Type': 'application/json'
  }
})

// Request interceptor
apiClient.interceptors.request.use(
  (config) => {
    console.log('[API Request]', config.method.toUpperCase(), config.url, config.data)
    return config
  },
  (error) => {
    console.error('[API Request Error]', error)
    return Promise.reject(error)
  }
)

// Response interceptor
apiClient.interceptors.response.use(
  (response) => {
    console.log('[API Response]', response.config.url, response.data)
    return response
  },
  (error) => {
    console.error('[API Response Error]', error.config?.url, error.message)
    if (error.response) {
      console.error('[API Error Response]', error.response.data)
    }
    return Promise.reject(error)
  }
)

export const rpcApi = {
  // Health check
  async health() {
    const response = await apiClient.get('/health')
    return response.data
  },

  // Get current state
  async getState() {
    const response = await apiClient.get('/rpc/state')
    return response.data
  },

  // Get statistics
  async getStats() {
    const response = await apiClient.get('/rpc/stats')
    return response.data
  },

  // Set task configuration
  async setConfig(taskConfig) {
    const response = await apiClient.post('/rpc/config', {
      task_config: taskConfig
    })
    return response.data
  },

  // Begin recording
  async begin(taskId) {
    const response = await apiClient.post('/rpc/begin', {
      task_id: taskId
    })
    return response.data
  },

  // Finish recording
  async finish(taskId) {
    const response = await apiClient.post('/rpc/finish', {
      task_id: taskId
    })
    return response.data
  },

  // Pause recording
  async pause() {
    const response = await apiClient.post('/rpc/pause')
    return response.data
  },

  // Resume recording
  async resume() {
    const response = await apiClient.post('/rpc/resume')
    return response.data
  },

  // Cancel recording
  async cancel(taskId) {
    const response = await apiClient.post('/rpc/cancel', {
      task_id: taskId
    })
    return response.data
  },

  // Clear configuration
  async clear() {
    const response = await apiClient.post('/rpc/clear')
    return response.data
  },

  // Quit program
  async quit() {
    const response = await apiClient.post('/rpc/quit')
    return response.data
  }
}
