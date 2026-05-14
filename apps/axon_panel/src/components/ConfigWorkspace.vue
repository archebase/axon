<template>
  <section class="config-workspace">
    <div class="workspace-header">
      <div>
        <p class="eyebrow">Robot Config</p>
        <h3>Configuration Workspace</h3>
        <p class="path-line">{{ configDirLabel }}</p>
      </div>

      <div class="workspace-actions">
        <span :class="['status-pill', injectionEnabled ? 'enabled' : 'disabled']">
          {{ injectionEnabled ? 'Injection On' : 'Injection Off' }}
        </span>
        <button type="button" class="ghost-btn" :disabled="busy" @click="refreshAll">
          Refresh
        </button>
        <button type="button" class="ghost-btn" :disabled="busy" @click="scanConfig(true)">
          Scan
        </button>
        <button type="button" class="primary-btn" :disabled="busy" @click="toggleInjection">
          {{ injectionEnabled ? 'Disable' : 'Enable' }}
        </button>
      </div>
    </div>

    <div v-if="loadError" class="alert error">{{ loadError }}</div>

    <div class="tabs" role="tablist" aria-label="Configuration workspace">
      <button
        v-for="tab in tabs"
        :key="tab.id"
        type="button"
        :class="{ active: activeTab === tab.id }"
        @click="activeTab = tab.id"
      >
        {{ tab.label }}
      </button>
    </div>

    <div v-if="activeTab === 'files'" class="tab-panel">
      <div class="summary-grid">
        <div class="metric">
          <span class="metric-value">{{ files.length }}</span>
          <span class="metric-label">Files</span>
        </div>
        <div class="metric">
          <span class="metric-value">{{ formatBytes(cacheStatus.total_size) }}</span>
          <span class="metric-label">Cache Size</span>
        </div>
        <div class="metric">
          <span class="metric-value">{{ cacheStatus.cache_exists ? 'Ready' : 'Missing' }}</span>
          <span class="metric-label">MCAP Cache</span>
        </div>
        <div class="metric">
          <span class="metric-value">{{ cacheStatus.last_scanned_at ? formatDate(cacheStatus.last_scanned_at) : 'Never' }}</span>
          <span class="metric-label">Last Scan</span>
        </div>
      </div>

      <form class="upload-bar" @submit.prevent="uploadConfig">
        <select v-model="uploadCategory" aria-label="Config category">
          <option value="urdf">URDF</option>
          <option value="calibration">Calibration</option>
          <option value="sensor">Sensor</option>
          <option value="config">Config</option>
        </select>
        <input
          ref="fileInput"
          type="file"
          @change="handleUploadFile"
        />
        <input
          v-model="uploadPath"
          type="text"
          placeholder="target path"
          aria-label="Target path"
        />
        <button type="submit" class="primary-btn" :disabled="busy || !uploadFile">
          Upload
        </button>
      </form>

      <div class="file-layout">
        <div class="file-list">
          <button
            v-for="file in files"
            :key="file.path"
            type="button"
            :class="['file-row', { selected: selectedPath === file.path, invalid: file.validation.errors.length }]"
            @click="selectFile(file)"
          >
            <span class="file-main">
              <span class="file-name">{{ file.path }}</span>
              <span class="file-meta">{{ file.category }} · {{ formatBytes(file.size) }}</span>
            </span>
            <span v-if="file.validation.errors.length" class="count-badge error">
              {{ file.validation.errors.length }}
            </span>
            <span v-else-if="file.validation.warnings.length" class="count-badge warning">
              {{ file.validation.warnings.length }}
            </span>
          </button>

          <div v-if="files.length === 0" class="empty-state">
            No config files
          </div>
        </div>

        <div class="preview-pane">
          <div v-if="selectedFile" class="preview-header">
            <div>
              <h4>{{ selectedFile.path }}</h4>
              <p>{{ selectedFile.content_type }} · {{ formatBytes(selectedFile.size) }}</p>
            </div>
            <span class="category-badge">{{ selectedFile.category }}</span>
          </div>

          <div v-if="selectedFile && selectedFile.validation.errors.length" class="field-errors">
            <div v-for="error in selectedFile.validation.errors" :key="error" class="alert error">
              {{ error }}
            </div>
          </div>

          <div v-if="selectedFile && selectedFile.validation.warnings.length" class="field-errors">
            <div v-for="warning in selectedFile.validation.warnings" :key="warning" class="alert warning">
              {{ warning }}
            </div>
          </div>

          <pre v-if="selectedContent && !selectedContent.binary" class="file-preview">{{ selectedContent.content }}</pre>
          <div v-else-if="selectedContent && selectedContent.binary" class="empty-state">
            Binary file preview unavailable
          </div>
          <div v-else class="empty-state">
            Select a config file
          </div>
        </div>
      </div>
    </div>

    <div v-if="activeTab === 'history'" class="tab-panel">
      <div class="history-layout">
        <div class="history-list">
          <button
            v-for="item in history"
            :key="item.id"
            type="button"
            :class="['history-row', { selected: selectedHistoryId === item.id }]"
            @click="selectHistory(item)"
          >
            <span class="history-action">{{ actionLabel(item.action) }}</span>
            <span class="history-path">{{ item.path || item.message || 'config cache' }}</span>
            <span class="history-time">{{ formatDate(item.created_at) }}</span>
          </button>
          <div v-if="history.length === 0" class="empty-state">
            No change history
          </div>
        </div>

        <div class="diff-pane">
          <div v-if="selectedHistory" class="preview-header">
            <div>
              <h4>{{ selectedHistory.path || actionLabel(selectedHistory.action) }}</h4>
              <p>{{ formatDate(selectedHistory.created_at) }}</p>
            </div>
          </div>
          <div v-if="diffError" class="alert warning">{{ diffError }}</div>
          <pre v-if="diffLines.length" class="diff-view"><span
            v-for="(line, index) in diffLines"
            :key="index"
            :class="['diff-line', line.type]"
          >{{ diffPrefix(line.type) }}{{ line.text }}
</span></pre>
          <div v-else class="empty-state">
            Select an update with a previous version
          </div>
        </div>
      </div>
    </div>

    <div v-if="activeTab === 'tasks'" class="tab-panel">
      <div class="task-toolbar">
        <span>{{ selectedTaskIds.length }} selected</span>
        <button type="button" class="ghost-btn" :disabled="busy || !selectedTaskIds.length" @click="batchTasks('mark_reviewed')">
          Mark Reviewed
        </button>
        <button type="button" class="ghost-btn" :disabled="busy || !selectedTaskIds.length" @click="batchTasks('clear_reviewed')">
          Clear Reviewed
        </button>
        <button type="button" class="primary-btn" :disabled="busy || !selectedTaskIds.length" @click="batchTasks('queue_upload')">
          Queue Upload
        </button>
      </div>

      <div class="task-table">
        <label v-for="task in tasks" :key="task.task_id" class="task-row">
          <input
            type="checkbox"
            :checked="selectedTaskIds.includes(task.task_id)"
            @change="toggleTask(task.task_id)"
          />
          <span class="task-id">{{ task.task_id }}</span>
          <span class="task-status">{{ task.status }}</span>
          <span>{{ formatBytes(task.size) }}</span>
          <span>{{ formatDate(task.modified_at) }}</span>
          <span :class="['sidecar', task.sidecar_exists ? 'ok' : 'missing']">
            {{ task.sidecar_exists ? 'sidecar' : 'no sidecar' }}
          </span>
        </label>
        <div v-if="tasks.length === 0" class="empty-state">
          No recording tasks
        </div>
      </div>
    </div>

    <div v-if="activeTab === 'diagnostics'" class="tab-panel diagnostics-panel">
      <div class="logging-row">
        <label>
          Log level
          <select v-model="logLevel" @change="setLogLevel">
            <option v-for="level in logLevels" :key="level" :value="level">
              {{ level }}
            </option>
          </select>
        </label>
        <button type="button" class="ghost-btn" :disabled="busy" @click="scanConfig(false)">
          Full Rebuild
        </button>
      </div>

      <div
        v-for="diagnostic in diagnostics"
        :key="`${diagnostic.code}-${diagnostic.path || ''}-${diagnostic.message}`"
        :class="['diagnostic', diagnostic.severity]"
      >
        <span class="diagnostic-severity">{{ diagnostic.severity }}</span>
        <span class="diagnostic-message">
          <strong v-if="diagnostic.path">{{ diagnostic.path }}:</strong>
          {{ diagnostic.message }}
        </span>
      </div>
      <div v-if="diagnostics.length === 0" class="empty-state">
        No diagnostics
      </div>
    </div>
  </section>
</template>

<script setup>
import { computed, onMounted, ref } from 'vue'
import { panelApi } from '../api/panel'

const emit = defineEmits(['log'])

const tabs = [
  { id: 'files', label: 'Files' },
  { id: 'history', label: 'History' },
  { id: 'tasks', label: 'Tasks' },
  { id: 'diagnostics', label: 'Diagnostics' }
]

const activeTab = ref('files')
const status = ref(null)
const tasks = ref([])
const busy = ref(false)
const loadError = ref('')
const uploadCategory = ref('urdf')
const uploadPath = ref('')
const uploadFile = ref(null)
const fileInput = ref(null)
const selectedPath = ref('')
const selectedContent = ref(null)
const selectedHistory = ref(null)
const selectedHistoryId = ref('')
const diffLines = ref([])
const diffError = ref('')
const selectedTaskIds = ref([])
const logLevel = ref('info')
const logLevels = ref(['debug', 'info', 'warning', 'error', 'fatal'])

const configDirLabel = computed(() => status.value?.config_dir || 'config directory unavailable')
const cacheStatus = computed(() => status.value?.cache || {})
const files = computed(() => status.value?.files || [])
const history = computed(() => status.value?.history || [])
const diagnostics = computed(() => status.value?.diagnostics || [])
const injectionEnabled = computed(() => Boolean(cacheStatus.value.enabled))
const selectedFile = computed(() => files.value.find(file => file.path === selectedPath.value))

function log(message, type = 'info') {
  emit('log', message, type)
}

function replaceStatus(nextStatus) {
  status.value = nextStatus
  if (selectedPath.value && !files.value.some(file => file.path === selectedPath.value)) {
    selectedPath.value = ''
    selectedContent.value = null
  }
}

function errorMessage(error) {
  return error?.response?.data?.message || error?.message || String(error)
}

async function refreshStatus() {
  replaceStatus(await panelApi.getConfigStatus())
}

async function refreshTasks() {
  const data = await panelApi.getTasks()
  tasks.value = data.tasks || []
  selectedTaskIds.value = selectedTaskIds.value.filter(id =>
    tasks.value.some(task => task.task_id === id)
  )
}

async function refreshLogging() {
  const data = await panelApi.getLogging()
  const logging = data.logging || {}
  logLevel.value = logging.level || 'info'
  logLevels.value = logging.levels || logLevels.value
}

async function refreshAll() {
  busy.value = true
  loadError.value = ''
  try {
    await Promise.all([refreshStatus(), refreshTasks(), refreshLogging()])
  } catch (error) {
    loadError.value = errorMessage(error)
    log(`Panel API failed: ${loadError.value}`, 'error')
  } finally {
    busy.value = false
  }
}

async function scanConfig(incremental) {
  busy.value = true
  loadError.value = ''
  try {
    const data = await panelApi.scanConfig({ incremental })
    replaceStatus(data.status || status.value)
    log(data.message || 'Config cache rebuilt', data.success ? 'success' : 'warning')
  } catch (error) {
    loadError.value = errorMessage(error)
    log(`Config scan failed: ${loadError.value}`, 'error')
  } finally {
    busy.value = false
  }
}

async function toggleInjection() {
  busy.value = true
  loadError.value = ''
  try {
    const data = await panelApi.setInjectionEnabled(!injectionEnabled.value)
    replaceStatus(data.status || status.value)
    log(data.message, 'success')
  } catch (error) {
    loadError.value = errorMessage(error)
    log(`Injection toggle failed: ${loadError.value}`, 'error')
  } finally {
    busy.value = false
  }
}

function handleUploadFile(event) {
  const [file] = event.target.files || []
  uploadFile.value = file || null
  if (file && !uploadPath.value) {
    const base = file.name.replace(/[^\w.-]/g, '_')
    if (uploadCategory.value === 'urdf') {
      uploadPath.value = `robot/${base}`
    } else if (uploadCategory.value === 'sensor') {
      uploadPath.value = `sensors/${base}`
    } else if (uploadCategory.value === 'calibration') {
      uploadPath.value = `calibration/${base}`
    } else {
      uploadPath.value = base
    }
  }
}

async function uploadConfig() {
  if (!uploadFile.value) {
    return
  }
  busy.value = true
  loadError.value = ''
  try {
    const data = await panelApi.uploadConfigFile({
      file: uploadFile.value,
      path: uploadPath.value,
      category: uploadCategory.value
    })
    replaceStatus(data.status || status.value)
    uploadFile.value = null
    uploadPath.value = ''
    if (fileInput.value) {
      fileInput.value.value = ''
    }
    log(data.message, 'success')
  } catch (error) {
    loadError.value = errorMessage(error)
    log(`Upload failed: ${loadError.value}`, 'error')
  } finally {
    busy.value = false
  }
}

async function selectFile(file) {
  selectedPath.value = file.path
  selectedContent.value = null
  try {
    selectedContent.value = await panelApi.getConfigFile(file.path)
  } catch (error) {
    selectedContent.value = {
      binary: false,
      content: `Failed to load file: ${errorMessage(error)}`
    }
  }
}

async function selectHistory(item) {
  selectedHistory.value = item
  selectedHistoryId.value = item.id
  diffLines.value = []
  diffError.value = ''
  if (!item.previous_snapshot) {
    diffError.value = 'No previous version for this event'
    return
  }
  try {
    const data = await panelApi.getConfigDiff(item.id)
    diffLines.value = data.lines || []
    if (data.truncated) {
      diffError.value = 'Diff truncated'
    }
  } catch (error) {
    diffError.value = errorMessage(error)
  }
}

function toggleTask(taskId) {
  if (selectedTaskIds.value.includes(taskId)) {
    selectedTaskIds.value = selectedTaskIds.value.filter(id => id !== taskId)
  } else {
    selectedTaskIds.value = [...selectedTaskIds.value, taskId]
  }
}

async function batchTasks(action) {
  if (!selectedTaskIds.value.length) {
    return
  }
  busy.value = true
  loadError.value = ''
  try {
    const data = await panelApi.batchTasks(action, selectedTaskIds.value)
    tasks.value = data.tasks || tasks.value
    selectedTaskIds.value = []
    log(data.message, 'success')
  } catch (error) {
    loadError.value = errorMessage(error)
    log(`Task batch failed: ${loadError.value}`, 'error')
  } finally {
    busy.value = false
  }
}

async function setLogLevel() {
  loadError.value = ''
  try {
    await panelApi.setLogLevel(logLevel.value)
    log(`Log level set to ${logLevel.value}`, 'success')
  } catch (error) {
    loadError.value = errorMessage(error)
    log(`Log level change failed: ${loadError.value}`, 'error')
  }
}

function formatBytes(bytes) {
  const value = Number(bytes || 0)
  if (value === 0) return '0 B'
  const units = ['B', 'KB', 'MB', 'GB', 'TB']
  const index = Math.min(Math.floor(Math.log(value) / Math.log(1024)), units.length - 1)
  return `${(value / Math.pow(1024, index)).toFixed(index === 0 ? 0 : 1)} ${units[index]}`
}

function formatDate(value) {
  if (!value) return 'n/a'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  return date.toLocaleString()
}

function actionLabel(action) {
  const labels = {
    upload: 'Upload',
    update: 'Update',
    scan: 'Scan',
    incremental_scan: 'Scan',
    enable_injection: 'Enable',
    disable_injection: 'Disable'
  }
  return labels[action] || action || 'Change'
}

function diffPrefix(type) {
  if (type === 'added') return '+ '
  if (type === 'removed') return '- '
  return '  '
}

onMounted(refreshAll)
</script>

<style scoped>
.config-workspace {
  background: var(--surface-color);
  border: 1px solid rgba(7, 51, 140, 0.08);
  border-radius: 8px;
  box-shadow: 0 10px 30px rgba(7, 51, 140, 0.08);
  padding: 1.25rem;
}

.workspace-header {
  display: flex;
  justify-content: space-between;
  gap: 1rem;
  align-items: flex-start;
}

.eyebrow {
  color: var(--highlight-color);
  font-size: 0.75rem;
  font-weight: 700;
  letter-spacing: 0;
  margin: 0 0 0.25rem;
  text-transform: uppercase;
}

h3, h4 {
  color: var(--primary-color);
  margin: 0;
}

.path-line {
  color: var(--text-secondary);
  font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
  font-size: 0.75rem;
  margin: 0.35rem 0 0;
  overflow-wrap: anywhere;
}

.workspace-actions,
.task-toolbar,
.logging-row {
  align-items: center;
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
  justify-content: flex-end;
}

button,
select,
input {
  font: inherit;
}

button {
  border-radius: 6px;
  cursor: pointer;
  font-weight: 600;
  min-height: 38px;
  padding: 0.55rem 0.8rem;
  transition: all 0.2s ease;
}

button:disabled {
  cursor: not-allowed;
  opacity: 0.5;
}

.primary-btn {
  background: var(--primary-color);
  border: 1px solid var(--primary-color);
  color: white;
}

.primary-btn:hover:not(:disabled) {
  background: #0b4db8;
}

.ghost-btn {
  background: rgba(7, 51, 140, 0.04);
  border: 1px solid rgba(7, 51, 140, 0.16);
  color: var(--primary-color);
}

.ghost-btn:hover:not(:disabled) {
  background: rgba(7, 51, 140, 0.08);
}

.status-pill,
.category-badge,
.count-badge,
.task-status,
.sidecar,
.diagnostic-severity {
  border-radius: 999px;
  font-size: 0.72rem;
  font-weight: 700;
  padding: 0.25rem 0.55rem;
  text-transform: uppercase;
}

.status-pill.enabled,
.sidecar.ok {
  background: rgba(16, 185, 129, 0.12);
  color: #047857;
}

.status-pill.disabled,
.sidecar.missing {
  background: rgba(245, 158, 11, 0.14);
  color: #b45309;
}

.tabs {
  border-bottom: 1px solid rgba(7, 51, 140, 0.12);
  display: flex;
  gap: 0.25rem;
  margin-top: 1rem;
}

.tabs button {
  background: transparent;
  border: 0;
  border-bottom: 3px solid transparent;
  border-radius: 0;
  color: var(--text-secondary);
}

.tabs button.active {
  border-bottom-color: var(--highlight-color);
  color: var(--primary-color);
}

.tab-panel {
  padding-top: 1rem;
}

.summary-grid {
  display: grid;
  gap: 0.75rem;
  grid-template-columns: repeat(4, minmax(0, 1fr));
}

.metric {
  background: rgba(7, 51, 140, 0.04);
  border: 1px solid rgba(7, 51, 140, 0.08);
  border-radius: 6px;
  padding: 0.75rem;
}

.metric-value {
  color: var(--primary-color);
  display: block;
  font-weight: 800;
}

.metric-label {
  color: var(--text-secondary);
  display: block;
  font-size: 0.72rem;
  margin-top: 0.2rem;
  text-transform: uppercase;
}

.upload-bar {
  align-items: center;
  background: #f8fafc;
  border: 1px solid rgba(7, 51, 140, 0.1);
  border-radius: 6px;
  display: grid;
  gap: 0.6rem;
  grid-template-columns: 140px minmax(180px, 1fr) minmax(180px, 1fr) auto;
  margin-top: 0.85rem;
  padding: 0.75rem;
}

select,
input[type='text'],
input[type='file'] {
  background: white;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  color: var(--text-primary);
  min-height: 38px;
  padding: 0.45rem 0.55rem;
  width: 100%;
}

.file-layout,
.history-layout {
  display: grid;
  gap: 1rem;
  grid-template-columns: minmax(250px, 0.9fr) minmax(320px, 1.4fr);
  margin-top: 1rem;
}

.file-list,
.history-list,
.preview-pane,
.diff-pane,
.task-table {
  border: 1px solid rgba(7, 51, 140, 0.1);
  border-radius: 6px;
  min-height: 260px;
  overflow: hidden;
}

.file-list,
.history-list,
.task-table {
  background: #f8fafc;
}

.file-row,
.history-row {
  align-items: center;
  background: transparent;
  border: 0;
  border-bottom: 1px solid rgba(7, 51, 140, 0.08);
  border-radius: 0;
  color: var(--text-primary);
  display: flex;
  gap: 0.6rem;
  justify-content: space-between;
  min-height: auto;
  padding: 0.75rem;
  text-align: left;
  width: 100%;
}

.file-row.selected,
.history-row.selected {
  background: rgba(7, 51, 140, 0.08);
}

.file-row.invalid {
  border-left: 3px solid var(--error-color);
}

.file-main {
  min-width: 0;
}

.file-name,
.history-path,
.task-id {
  display: block;
  font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
  overflow-wrap: anywhere;
}

.file-meta,
.history-time,
.preview-header p {
  color: var(--text-secondary);
  font-size: 0.75rem;
}

.count-badge.error,
.diagnostic.error .diagnostic-severity {
  background: rgba(239, 68, 68, 0.12);
  color: #b91c1c;
}

.count-badge.warning,
.diagnostic.warning .diagnostic-severity {
  background: rgba(245, 158, 11, 0.16);
  color: #b45309;
}

.diagnostic.info .diagnostic-severity {
  background: rgba(7, 157, 217, 0.14);
  color: var(--primary-color);
}

.preview-pane,
.diff-pane {
  background: white;
  display: flex;
  flex-direction: column;
}

.preview-header {
  align-items: center;
  border-bottom: 1px solid rgba(7, 51, 140, 0.1);
  display: flex;
  justify-content: space-between;
  padding: 0.85rem;
}

.category-badge {
  background: rgba(7, 157, 217, 0.14);
  color: var(--primary-color);
}

.file-preview,
.diff-view {
  background: #0f172a;
  color: #dbeafe;
  flex: 1;
  font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
  font-size: 0.78rem;
  line-height: 1.5;
  margin: 0;
  min-height: 220px;
  overflow: auto;
  padding: 1rem;
  white-space: pre-wrap;
}

.diff-line {
  display: block;
}

.diff-line.added {
  background: rgba(16, 185, 129, 0.16);
  color: #bbf7d0;
}

.diff-line.removed {
  background: rgba(239, 68, 68, 0.16);
  color: #fecaca;
}

.alert {
  border-radius: 6px;
  margin-top: 0.75rem;
  padding: 0.65rem 0.8rem;
}

.alert.error {
  background: rgba(239, 68, 68, 0.1);
  color: #991b1b;
}

.alert.warning {
  background: rgba(245, 158, 11, 0.13);
  color: #92400e;
}

.field-errors {
  padding: 0 0.85rem;
}

.empty-state {
  color: var(--text-secondary);
  padding: 2rem 1rem;
  text-align: center;
}

.history-action {
  color: var(--highlight-color);
  font-weight: 800;
  min-width: 4rem;
}

.task-toolbar {
  justify-content: flex-start;
  margin-bottom: 0.75rem;
}

.task-row {
  align-items: center;
  border-bottom: 1px solid rgba(7, 51, 140, 0.08);
  color: var(--text-primary);
  display: grid;
  gap: 0.7rem;
  grid-template-columns: 28px minmax(140px, 1fr) 100px 90px 150px 100px;
  padding: 0.75rem;
}

.task-row input {
  width: auto;
}

.task-status {
  background: rgba(7, 51, 140, 0.08);
  color: var(--primary-color);
  text-align: center;
}

.diagnostics-panel {
  display: grid;
  gap: 0.65rem;
}

.logging-row {
  justify-content: space-between;
}

.logging-row label {
  align-items: center;
  color: var(--text-secondary);
  display: flex;
  font-weight: 700;
  gap: 0.6rem;
}

.diagnostic {
  align-items: center;
  border: 1px solid rgba(7, 51, 140, 0.08);
  border-radius: 6px;
  display: flex;
  gap: 0.7rem;
  padding: 0.75rem;
}

.diagnostic.error {
  background: rgba(239, 68, 68, 0.06);
}

.diagnostic.warning {
  background: rgba(245, 158, 11, 0.07);
}

.diagnostic.info {
  background: rgba(7, 157, 217, 0.06);
}

.diagnostic-message {
  overflow-wrap: anywhere;
}

@media (max-width: 980px) {
  .workspace-header,
  .logging-row {
    align-items: stretch;
    flex-direction: column;
  }

  .workspace-actions {
    justify-content: flex-start;
  }

  .summary-grid,
  .upload-bar,
  .file-layout,
  .history-layout {
    grid-template-columns: 1fr;
  }

  .task-row {
    grid-template-columns: 28px minmax(120px, 1fr);
  }

  .task-row span:nth-child(n + 4) {
    display: none;
  }
}
</style>
