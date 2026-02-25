<template>
  <div id="app">
    <header class="header">
      <h1>AxonPanel</h1>
      <div class="header-actions">
        <ConnectionStatus
          :connected="connected"
          :health="health"
          @refresh="fetchHealth"
        />
        <button class="log-toggle-btn" @click="showLogs = !showLogs" :title="showLogs ? 'Hide logs' : 'Show logs'">
          <span v-if="logs.length > 0" class="log-badge">{{ logs.length }}</span>
          <span class="log-icon">{{ showLogs ? '📋' : '📝' }}</span>
        </button>
      </div>
    </header>

    <main class="main">
      <div class="panels">
        <!-- State Panel -->
        <StatePanel
          :state="currentState"
          :task-config="taskConfig"
          :stats="stats"
          @refresh="fetchState"
        />

        <!-- Control Panel -->
        <ControlPanel
          :state="currentState"
          :task-id="currentTaskId"
          @command="handleCommand"
        />

        <!-- Config Panel -->
        <ConfigPanel
          v-if="showConfigPanel"
          :state="currentState"
          @submit="handleConfigSubmit"
          @cancel="showConfigPanel = false"
        />
      </div>
    </main>

    <!-- Floating Log Panel -->
    <transition name="slide">
      <div v-if="showLogs" class="log-panel-floating">
        <div class="log-panel-header">
          <h3>Activity Log</h3>
          <div class="header-actions">
            <button class="clear-btn" @click="clearLogs" title="Clear logs">🗑️</button>
            <button class="close-btn" @click="showLogs = false" title="Close">×</button>
          </div>
        </div>
        <LogPanel :logs="logs" />
      </div>
    </transition>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { rpcApi } from './api/rpc'
import ConnectionStatus from './components/ConnectionStatus.vue'
import StatePanel from './components/StatePanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import ConfigPanel from './components/ConfigPanel.vue'
import LogPanel from './components/LogPanel.vue'

const connected = ref(false)
const health = ref(null)
const currentState = ref('idle')
const taskConfig = ref(null)
const currentTaskId = ref('')
const stats = ref(null)
const showConfigPanel = ref(false)
const showLogs = ref(true)
const logs = ref([])

let pollInterval = null

function addLog(message, type = 'info') {
  const timestamp = new Date().toLocaleTimeString()
  logs.value.unshift({ timestamp, message, type })
  // Keep only last 100 logs
  if (logs.value.length > 100) {
    logs.value.pop()
  }
}

async function fetchHealth() {
  try {
    const data = await rpcApi.health()
    health.value = data
    connected.value = true
  } catch (error) {
    connected.value = false
    addLog(`Health check failed: ${error.message}`, 'error')
  }
}

async function fetchState() {
  try {
    const data = await rpcApi.getState()
    if (data.success) {
      const previousState = currentState.value
      currentState.value = data.data.state

      // Clear local task config when transitioning to IDLE
      if (currentState.value === 'idle' && previousState !== 'idle') {
        taskConfig.value = null
        currentTaskId.value = ''
        addLog('Recording finished. Local task config cleared.', 'info')
      }
    }
  } catch (error) {
    addLog(`Failed to fetch state: ${error.message}`, 'error')
  }
}

async function fetchStats() {
  try {
    const data = await rpcApi.getStats()
    if (data.success) {
      stats.value = data.data
    }
  } catch (error) {
    // Don't log stats errors as they're less critical
  }
}

function clearLogs() {
  logs.value = []
}

async function handleCommand(command) {
  addLog(`Executing command: ${command}`, 'info')

  try {
    let result
    switch (command) {
      case 'config':
        showConfigPanel.value = true
        return

      case 'begin':
        addLog(`Using task_id: ${currentTaskId.value}`, 'info')
        if (!currentTaskId.value) {
          addLog('Error: No task_id available. Please configure a task first.', 'error')
          return
        }
        result = await rpcApi.begin(currentTaskId.value)
        break

      case 'finish':
        result = await rpcApi.finish(currentTaskId.value)
        break

      case 'pause':
        result = await rpcApi.pause()
        break

      case 'resume':
        result = await rpcApi.resume()
        break

      case 'cancel':
        result = await rpcApi.cancel(currentTaskId.value)
        break

      case 'clear':
        result = await rpcApi.clear()
        break

      case 'quit':
        result = await rpcApi.quit()
        addLog('Quit command sent. Server will shutdown.', 'warning')
        break

      default:
        addLog(`Unknown command: ${command}`, 'error')
        return
    }

    if (result.success) {
      addLog(result.message, 'success')
      await fetchState()
    } else {
      addLog(result.message, 'error')
    }
  } catch (error) {
    addLog(`Command failed: ${error.message}`, 'error')
  }
}

async function handleConfigSubmit(config) {
  addLog('Setting configuration...', 'info')
  addLog(`Task ID: ${config.task_id}`, 'info')
  try {
    const result = await rpcApi.setConfig(config)
    if (result.success) {
      addLog(result.message, 'success')

      // Store the task config locally in the frontend
      taskConfig.value = config
      currentTaskId.value = config.task_id

      showConfigPanel.value = false
      await fetchState()
      addLog(`Task config cached locally. Current task_id: ${currentTaskId.value}`, 'info')
    } else {
      addLog(result.message, 'error')
    }
  } catch (error) {
    addLog(`Config failed: ${error.message}`, 'error')
  }
}

function startPolling() {
  // Poll state every second
  pollInterval = setInterval(async () => {
    await fetchState()
    await fetchStats()
  }, 1000)
}

function stopPolling() {
  if (pollInterval) {
    clearInterval(pollInterval)
    pollInterval = null
  }
}

onMounted(async () => {
  await fetchHealth()
  await fetchState()
  startPolling()
})

onUnmounted(() => {
  stopPolling()
})
</script>

<style>
 :root {
   --primary-color: #07338c;
   --secondary-color: #079dd9;
   --accent-color: #27def2;
   --highlight-color: #d97706;
   --success-color: #10b981;
   --warning-color: #f59e0b;
   --error-color: #ef4444;
   --neutral-color: #64748b;
   --surface-color: #ffffff;
   --surface-tint: #f0f7ff;
   --surface-muted: #e0f2fe;
   --text-primary: #0f172a;
   --text-secondary: #334155;
   --border-color: #cbd5e1;
 }

 body {
   background: var(--surface-tint);
   color: var(--text-primary);
 }
/* Global styles to ensure edge labels are black */
.vue-flow__edgeLabel text,
.vue-flow__edgeLabel tspan,
.vue-flow__edge-textwrapper text,
.vue-flow__edge-textwrapper tspan {
  fill: var(--text-primary) !important;
  color: var(--text-primary) !important;
}
</style>

<style scoped>
#app {
  min-height: 100vh;
  background: var(--surface-tint);
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
}

.header {
  background: var(--primary-color);
  color: white;
  padding: 1.5rem 2rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.header h1 {
  margin: 0;
  font-size: 1.5rem;
  font-weight: 600;
}

.header-actions {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.log-toggle-btn {
  background: rgba(255, 255, 255, 0.16);
  border: 1px solid rgba(255, 255, 255, 0.28);
  color: white;
  padding: 0;
  border-radius: 6px;
  cursor: pointer;
  font-size: 0.875rem;
  font-weight: 500;
  transition: all 0.2s;
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
  /* Touch-friendly size */
  width: 44px;
  height: 44px;
  user-select: none;
  -webkit-tap-highlight-color: transparent;
}

.log-icon {
  font-size: 1.25rem;
  line-height: 1;
}

.log-toggle-btn:hover {
  background: rgba(255, 255, 255, 0.26);
  transform: translateY(-1px);
}

.log-toggle-btn:active {
  transform: translateY(0);
  background: rgba(255, 255, 255, 0.22);
}

.log-badge {
  position: absolute;
  top: -4px;
  right: -4px;
  background: var(--error-color);
  color: white;
  font-size: 0.65rem;
  padding: 0.125rem 0.375rem;
  border-radius: 999px;
  font-weight: 600;
  min-width: 1rem;
  height: 1rem;
  text-align: center;
  line-height: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 2px solid rgba(255, 255, 255, 0.35);
}

.log-panel-floating {
  position: fixed;
  bottom: 2rem;
  right: 2rem;
  width: 600px;
  height: 500px;
  background: var(--surface-color);
  border-radius: 12px;
  box-shadow: 0 10px 40px rgba(0,0,0,0.2);
  display: flex;
  flex-direction: column;
  overflow: hidden;
  z-index: 1000;
  resize: none;
}

/* Mobile responsive */
@media (max-width: 768px) {
  .log-panel-floating {
    position: fixed;
    bottom: 0;
    right: 0;
    left: 0;
    width: 100%;
    height: 60vh;
    border-radius: 12px 12px 0 0;
  }

  .header {
    padding: 1rem;
    flex-direction: column;
    gap: 1rem;
    align-items: flex-start;
  }

  .header h1 {
    font-size: 1.25rem;
  }

  .header-actions {
    width: 100%;
    justify-content: space-between;
  }

  .main {
    padding: 1rem;
  }

  .panels {
    grid-template-columns: 1fr;
    gap: 1rem;
  }
}

.log-panel-header {
  background: var(--primary-color);
  color: white;
  padding: 1rem 1.5rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(255,255,255,0.18);
}

.log-panel-header h3 {
  margin: 0;
  font-size: 1rem;
  font-weight: 600;
}

.header-actions {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.clear-btn {
  background: rgba(255, 255, 255, 0.16);
  border: none;
  color: white;
  font-size: 1.1rem;
  line-height: 1;
  width: 2.5rem;
  height: 2.5rem;
  border-radius: 6px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background 0.2s;
  min-width: 44px;
  min-height: 44px;
  user-select: none;
  -webkit-tap-highlight-color: transparent;
}

.clear-btn:hover {
  background: rgba(255, 255, 255, 0.26);
}

.clear-btn:active {
  background: rgba(255, 255, 255, 0.22);
}

.close-btn {
  background: rgba(255, 255, 255, 0.16);
  border: none;
  color: white;
  font-size: 1.5rem;
  line-height: 1;
  width: 2.5rem;
  height: 2.5rem;
  border-radius: 6px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background 0.2s;
  /* Touch-friendly size */
  min-width: 44px;
  min-height: 44px;
  user-select: none;
  -webkit-tap-highlight-color: transparent;
}

.close-btn:hover {
  background: rgba(255, 255, 255, 0.26);
}

.close-btn:active {
  background: rgba(255, 255, 255, 0.22);
}

/* Slide animation */
.slide-enter-active,
.slide-leave-active {
  transition: all 0.3s ease;
}

.slide-enter-from {
  opacity: 0;
  transform: translateY(20px) scale(0.95);
}

.slide-leave-to {
  opacity: 0;
  transform: translateY(20px) scale(0.95);
}

.main {
  padding: 2rem;
  max-width: 1400px;
  margin: 0 auto;
}

.panels {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
  gap: 1.5rem;
  margin-bottom: 1.5rem;
}
</style>
