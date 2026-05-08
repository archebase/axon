import axios from 'axios'

let cachedBaseUrl = null

function getBaseUrl() {
  // Return cached value if already computed
  if (cachedBaseUrl !== null) {
    return cachedBaseUrl
  }

  // Explicit API base URL override (full URL)
  if (import.meta.env.VITE_API_BASE_URL) {
    cachedBaseUrl = import.meta.env.VITE_API_BASE_URL
    return cachedBaseUrl
  }
  // Dev mode: use relative URL so Vite proxy forwards /rpc → recorder
  if (import.meta.env.DEV) {
    cachedBaseUrl = ''
    return cachedBaseUrl
  }
  // Production: check runtime config from server first
  // window.__AXON_CONFIG__ is set by /config.js served from the C++ server
  if (typeof window.__AXON_CONFIG__ !== 'undefined') {
    const port = window.__AXON_CONFIG__
    cachedBaseUrl = `${window.location.protocol}//${window.location.hostname}:${port}`
    return cachedBaseUrl
  }
  // Fallback: derive from panel port (default: panel port - 2)
  const recorderPort = (parseInt(window.location.port, 10) || 8082) - 2
  cachedBaseUrl = `${window.location.protocol}//${window.location.hostname}:${recorderPort}`
  return cachedBaseUrl
}

// Create axios instance - baseURL is evaluated lazily via interceptor
const apiClient = axios.create({
  headers: {
    'Content-Type': 'application/json'
  }
})

// Inject baseURL on each request (lazy evaluation after config.js loads)
apiClient.interceptors.request.use(
  (config) => {
    config.baseURL = getBaseUrl()
    return config
  },
  (error) => {
    return Promise.reject(error)
  }
)

// Response interceptor
apiClient.interceptors.response.use(
  (response) => {
    return response
  },
  (error) => {
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
