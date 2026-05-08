<template>
  <div class="panel">
    <h3>Statistics</h3>

    <div v-if="taskConfig" class="task-info">
      <h4>Task Configuration</h4>
      <div class="info-grid">
        <div class="info-item">
          <span class="key">Task ID:</span>
          <span class="value">{{ taskConfig.task_id }}</span>
        </div>
        <div class="info-item">
          <span class="key">Device ID:</span>
          <span class="value">{{ taskConfig.device_id }}</span>
        </div>
        <div v-if="taskConfig.scene" class="info-item">
          <span class="key">Scene:</span>
          <span class="value">{{ taskConfig.scene }}</span>
        </div>
        <div v-if="taskConfig.factory" class="info-item">
          <span class="key">Factory:</span>
          <span class="value">{{ taskConfig.factory }}</span>
        </div>
        <div v-if="taskConfig.topics" class="info-item full-width">
          <span class="key">Topics:</span>
          <div class="topics-list">
            <span v-for="topic in taskConfig.topics" :key="topic" class="topic-tag">
              {{ topic }}
            </span>
          </div>
        </div>
      </div>
    </div>

    <div v-if="stats" class="stats">
      <h4>Recording Statistics</h4>
      <div class="stats-grid">
        <div class="stat-item">
          <div class="stat-value">{{ formatNumber(stats.messages_received) }}</div>
          <div class="stat-label">Received</div>
        </div>
        <div class="stat-item">
          <div class="stat-value">{{ formatNumber(stats.messages_written) }}</div>
          <div class="stat-label">Written</div>
        </div>
        <div class="stat-item">
          <div class="stat-value">{{ formatNumber(stats.messages_dropped) }}</div>
          <div class="stat-label">Dropped</div>
        </div>
        <div class="stat-item">
          <div class="stat-value">{{ formatBytes(stats.bytes_written) }}</div>
          <div class="stat-label">Size</div>
        </div>
      </div>
    </div>

    <button @click="$emit('refresh')" class="refresh-btn">
      ⟳ Refresh
    </button>
  </div>
</template>

<script setup>
const props = defineProps({
  state: String,
  taskConfig: Object,
  stats: Object
})

defineEmits(['refresh'])

function formatNumber(num) {
  return new Intl.NumberFormat().format(num)
}

function formatBytes(bytes) {
  if (bytes === 0) return '0 B'
  const k = 1024
  const sizes = ['B', 'KB', 'MB', 'GB', 'TB']
  const i = Math.floor(Math.log(bytes) / Math.log(k))
  return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i]
}
</script>

<style scoped>
.panel {
  background: var(--surface-color);
  border-radius: 8px;
  padding: 1.5rem;
  box-shadow: 0 10px 30px rgba(7, 51, 140, 0.08);
  border: 1px solid rgba(7, 51, 140, 0.08);
}

h3, h4 {
  margin: 0 0 1rem 0;
  font-weight: 600;
  color: var(--primary-color);
}

h4 {
  font-size: 0.95rem;
  margin-top: 1rem;
}

.task-info, .stats {
  margin-top: 1rem;
  padding-top: 1rem;
  border-top: 1px solid rgba(7, 51, 140, 0.15);
}

.info-grid {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 0.75rem;
  align-items: center;
}

.info-item {
  display: contents;
}

.info-item.full-width {
  grid-column: 1 / -1;
  display: block;
}

.key {
  font-weight: 500;
  color: var(--text-secondary);
  font-size: 0.875rem;
}

.value {
  color: var(--text-primary);
  font-family: monospace;
  font-size: 0.875rem;
  word-break: break-all;
}

.topics-list {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
  margin-top: 0.5rem;
}

.topic-tag {
  background: rgba(7, 51, 140, 0.08);
  color: var(--primary-color);
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-family: monospace;
  font-size: 0.75rem;
}

.stats-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}

.stat-item {
  background: rgba(7, 51, 140, 0.04);
  padding: 1rem;
  border-radius: 6px;
  text-align: center;
}

.stat-value {
  font-size: 1.25rem;
  font-weight: 700;
  color: var(--primary-color);
  margin-bottom: 0.25rem;
}

.stat-label {
  font-size: 0.75rem;
  color: var(--text-secondary);
  text-transform: uppercase;
}

.refresh-btn {
  margin-top: 1rem;
  width: 100%;
  padding: 0.75rem;
  background: rgba(7, 51, 140, 0.08);
  border: 1px solid rgba(7, 51, 140, 0.2);
  color: var(--primary-color);
  border-radius: 6px;
  cursor: pointer;
  font-weight: 500;
  transition: all 0.2s;
}

.refresh-btn:hover {
  background: rgba(7, 51, 140, 0.12);
}

/* Mobile responsive */
@media (max-width: 768px) {
  .panel {
    padding: 1rem;
  }

  h3, h4 {
    font-size: 1rem;
    margin-bottom: 0.875rem;
  }

  .info-grid {
    grid-template-columns: 1fr;
    gap: 0.5rem;
  }

  .key, .value {
    font-size: 0.8rem;
  }

  .stats-grid {
    grid-template-columns: repeat(2, 1fr);
    gap: 0.75rem;
  }

  .stat-item {
    padding: 0.75rem;
  }

  .stat-value {
    font-size: 1.1rem;
  }

  .stat-label {
    font-size: 0.7rem;
  }

  .topic-tag {
    font-size: 0.7rem;
    padding: 0.2rem 0.4rem;
  }

  .refresh-btn {
    padding: 0.875rem;
    font-size: 0.9rem;
    min-height: 48px;
    user-select: none;
    -webkit-tap-highlight-color: transparent;
  }

  .refresh-btn:active {
    transform: scale(0.98);
  }
}
</style>
