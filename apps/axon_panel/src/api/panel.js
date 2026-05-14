import axios from 'axios'

const panelClient = axios.create({
  baseURL: '',
  headers: {
    Accept: 'application/json'
  }
})

function unwrap(response) {
  return response.data
}

export const panelApi = {
  async getConfigStatus() {
    return unwrap(await panelClient.get('/api/panel/config/status'))
  },

  async scanConfig({ incremental = true } = {}) {
    return unwrap(await panelClient.post('/api/panel/config/scan', null, {
      params: { incremental: incremental ? 'true' : 'false' }
    }))
  },

  async setInjectionEnabled(enabled) {
    const endpoint = enabled
      ? '/api/panel/config/injection/enable'
      : '/api/panel/config/injection/disable'
    return unwrap(await panelClient.post(endpoint))
  },

  async uploadConfigFile({ file, path, category }) {
    const form = new FormData()
    form.append('file', file)
    if (path) {
      form.append('path', path)
    }
    if (category) {
      form.append('category', category)
    }
    return unwrap(await panelClient.post('/api/panel/config/files', form))
  },

  async getConfigFile(path) {
    return unwrap(await panelClient.get('/api/panel/config/file', {
      params: { path }
    }))
  },

  async getPreviousConfigFile(historyId) {
    return unwrap(await panelClient.get('/api/panel/config/file', {
      params: { version: 'previous', history_id: historyId }
    }))
  },

  async getConfigDiff(historyId) {
    return unwrap(await panelClient.get('/api/panel/config/diff', {
      params: { history_id: historyId }
    }))
  },

  async getTasks() {
    return unwrap(await panelClient.get('/api/panel/tasks'))
  },

  async batchTasks(action, taskIds) {
    return unwrap(await panelClient.post('/api/panel/tasks/batch', {
      action,
      task_ids: taskIds
    }))
  },

  async getLogging() {
    return unwrap(await panelClient.get('/api/panel/logging'))
  },

  async setLogLevel(level) {
    return unwrap(await panelClient.post('/api/panel/logging/level', { level }))
  }
}
