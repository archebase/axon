<template>
  <div class="log-panel-content">
    <div class="log-container">
      <div
        v-for="(log, index) in logs"
        :key="index"
        :class="['log-entry', `log-${log.type}`]"
      >
        <span class="log-time">{{ log.timestamp }}</span>
        <span class="log-message">{{ log.message }}</span>
      </div>

      <div v-if="logs.length === 0" class="log-empty">
        No activity logs yet
      </div>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  logs: Array
})
</script>

<style scoped>
.log-panel-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.log-container {
  flex: 1;
  background: #1f2937;
  padding: 1rem;
  overflow-y: auto;
  font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
  font-size: 0.875rem;
}

.log-entry {
  display: flex;
  gap: 0.75rem;
  padding: 0.375rem 0;
  border-bottom: 1px solid rgba(255,255,255,0.05);
}

.log-entry:last-child {
  border-bottom: none;
}

.log-time {
  color: #9ca3af;
  flex-shrink: 0;
}

.log-message {
  color: #e5e7eb;
}

.log-info .log-message {
  color: #60a5fa;
}

.log-success .log-message {
  color: #34d399;
}

.log-warning .log-message {
  color: #fbbf24;
}

.log-error .log-message {
  color: #f87171;
}

.log-empty {
  color: #6b7280;
  text-align: center;
  padding: 2rem;
  font-style: italic;
}
</style>
