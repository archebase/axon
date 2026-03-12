<template>
  <div class="rpc-mode-selector">
    <span class="label">API type:</span>
    <select
      :value="modelValue"
      @change="$emit('update:modelValue', $event.target.value)"
      class="mode-select"
      title="Select RPC mode"
    >
      <option value="ws">WebSocket</option>
      <option value="http">HTTP</option>
    </select>
  </div>
</template>

<script setup>
defineProps({
  modelValue: {
    type: String,
    default: 'ws'
  }
})

defineEmits(['update:modelValue'])
</script>

<style scoped>
.rpc-mode-selector {
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 0.5rem;
  width: 100%;
}

.label {
  font-weight: 500;
  color: var(--text-secondary);
  font-size: 0.875rem;
}

.mode-select {
  padding: 0.375rem 2rem 0.375rem 0.75rem;
  background: var(--surface-color);
  border: 1px solid var(--border-color, rgba(7, 51, 140, 0.15));
  border-radius: 6px;
  color: var(--text-primary);
  font-size: 0.8rem;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
  appearance: none;
  background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='12' viewBox='0 0 12 12'%3E%3Cpath fill='%2307338c' d='M6 9L1 4h10z'/%3E%3C/svg%3E");
  background-repeat: no-repeat;
  background-position: right 0.5rem center;
  padding-right: 1.5rem;
}

.mode-select:hover {
  border-color: var(--primary-color);
  box-shadow: 0 0 0 2px rgba(7, 51, 140, 0.1);
}

.mode-select:focus {
  outline: none;
  border-color: var(--primary-color);
  box-shadow: 0 0 0 3px rgba(7, 51, 140, 0.15);
}

/* Mobile responsive */
@media (max-width: 768px) {
  .rpc-mode-selector {
    flex-wrap: wrap;
    gap: 0.375rem;
  }

  .label {
    font-size: 0.75rem;
  }

  .mode-select {
    padding: 0.3rem 2rem 0.3rem 0.6rem;
    font-size: 0.75rem;
  }
}
</style>
