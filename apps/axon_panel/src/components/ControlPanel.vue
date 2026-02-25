<template>
  <div class="panel">
    <h3>Control Panel</h3>

    <div class="control-buttons">
      <button
        @click="$emit('command', 'config')"
        :disabled="state === 'recording' || state === 'paused'"
        :class="{ active: state === 'ready' }"
        title="Set task configuration"
      >
        <span class="icon">⚙</span>
        <span>Config</span>
      </button>

      <button
        @click="$emit('command', 'begin')"
        :disabled="state !== 'ready'"
        :class="{ active: state === 'ready' }"
        title="Start recording"
      >
        <span class="icon">▶</span>
        <span>Begin</span>
      </button>

      <button
        @click="$emit('command', 'pause')"
        :disabled="state !== 'recording'"
        :class="{ active: state === 'recording' }"
        title="Pause recording"
      >
        <span class="icon">⏸</span>
        <span>Pause</span>
      </button>

      <button
        @click="$emit('command', 'resume')"
        :disabled="state !== 'paused'"
        :class="{ active: state === 'paused' }"
        title="Resume recording"
      >
        <span class="icon">▶</span>
        <span>Resume</span>
      </button>

      <button
        @click="$emit('command', 'finish')"
        :disabled="state !== 'recording' && state !== 'paused'"
        title="Finish recording"
      >
        <span class="icon">⏹</span>
        <span>Finish</span>
      </button>

      <button
        @click="$emit('command', 'cancel')"
        :disabled="state !== 'recording' && state !== 'paused'"
        title="Cancel recording"
        class="danger"
      >
        <span class="icon">✕</span>
        <span>Cancel</span>
      </button>

      <button
        @click="$emit('command', 'clear')"
        :disabled="state !== 'ready'"
        title="Clear configuration"
      >
        <span class="icon">🗑</span>
        <span>Clear</span>
      </button>

      <button
        @click="$emit('command', 'quit')"
        title="Quit program"
        class="danger"
      >
        <span class="icon">⏻</span>
        <span>Quit</span>
      </button>
    </div>

    <div v-if="taskId" class="task-id-display">
      <span class="label">Current Task ID:</span>
      <span class="task-id">{{ taskId }}</span>
    </div>

    <div class="state-transitions">
      <h4>State Machine</h4>
      <StateMachineDiagram :current-state="state" />
    </div>
  </div>
</template>

<script setup>
import StateMachineDiagram from './StateMachineDiagram.vue'

defineProps({
  state: String,
  taskId: String
})

defineEmits(['command'])
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
  margin-top: 1.5rem;
}

.control-buttons {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 0.75rem;
}

.control-buttons button {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  padding: 0.875rem 1rem;
  background: rgba(7, 51, 140, 0.04);
  border: 2px solid rgba(7, 51, 140, 0.12);
  color: var(--text-primary);
  border-radius: 8px;
  cursor: pointer;
  font-weight: 500;
  font-size: 0.9rem;
  transition: all 0.2s;
}

.control-buttons button:hover:not(:disabled) {
  background: rgba(7, 51, 140, 0.08);
  border-color: rgba(7, 51, 140, 0.2);
}

.control-buttons button:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}

.control-buttons button.active {
  background: rgba(7, 51, 140, 0.12);
  border-color: var(--primary-color);
  color: var(--primary-color);
}

.control-buttons button.danger {
  background: rgba(239, 68, 68, 0.08);
  border-color: rgba(239, 68, 68, 0.3);
  color: #b91c1c;
}

.control-buttons button.danger:hover:not(:disabled) {
  background: rgba(239, 68, 68, 0.12);
  border-color: rgba(239, 68, 68, 0.4);
}

.icon {
  font-size: 1.1rem;
}

.task-id-display {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(7, 51, 140, 0.04);
  border-radius: 6px;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.label {
  font-weight: 500;
  color: var(--text-secondary);
  font-size: 0.875rem;
}

.task-id {
  font-family: monospace;
  color: var(--text-primary);
  word-break: break-all;
}

.state-transitions {
  margin-top: 1.5rem;
}

/* Mobile responsive */
@media (max-width: 768px) {
  .panel {
    padding: 1rem;
  }

  .control-buttons {
    grid-template-columns: repeat(2, 1fr);
    gap: 0.5rem;
  }

  .control-buttons button {
    padding: 1rem 0.75rem;
    font-size: 0.85rem;
    min-height: 50px;
    user-select: none;
    -webkit-tap-highlight-color: transparent;
  }

  .control-buttons button:active:not(:disabled) {
    transform: scale(0.98);
  }

  .icon {
    font-size: 1rem;
  }
}
</style>
