<template>
  <div class="connection-status">
    <div class="status-item">
      <span class="label">Connection:</span>
      <span :class="['badge', connected ? 'connected' : 'disconnected']">
        {{ connected ? 'Connected' : 'Disconnected' }}
      </span>
    </div>
    <div v-if="health" class="status-item">
      <span class="label">Version:</span>
      <span class="value">{{ health.version }}</span>
    </div>
    <button @click="$emit('refresh')" class="refresh-btn" title="Refresh">
      ⟳
    </button>
  </div>
</template>

<script setup>
defineProps({
  connected: Boolean,
  health: Object
})

defineEmits(['refresh'])
</script>

<style scoped>
.connection-status {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.label {
  font-weight: 500;
  opacity: 0.9;
}

.badge {
  padding: 0.25rem 0.75rem;
  border-radius: 12px;
  font-size: 0.875rem;
  font-weight: 600;
  text-transform: uppercase;
}

.badge.connected {
  background: #10b981;
}

.badge.disconnected {
  background: #ef4444;
}

.value {
  font-family: monospace;
  background: rgba(255,255,255,0.2);
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
}

.refresh-btn {
  background: rgba(255,255,255,0.2);
  border: none;
  color: white;
  width: 36px;
  height: 36px;
  border-radius: 50%;
  cursor: pointer;
  font-size: 1.2rem;
  transition: background 0.2s;
}

.refresh-btn:hover {
  background: rgba(255,255,255,0.3);
}

.refresh-btn:active {
  transform: scale(0.95);
}

/* Mobile responsive */
@media (max-width: 768px) {
  .connection-status {
    flex-wrap: wrap;
    gap: 0.75rem;
  }

  .status-item {
    font-size: 0.85rem;
  }

  .label {
    display: none;
  }

  .badge {
    padding: 0.2rem 0.6rem;
    font-size: 0.75rem;
  }

  .value {
    font-size: 0.75rem;
    padding: 0.2rem 0.4rem;
  }

  .refresh-btn {
    width: 40px;
    height: 40px;
    font-size: 1rem;
    min-width: 44px;
    min-height: 44px;
    user-select: none;
    -webkit-tap-highlight-color: transparent;
  }
}
</style>
