<template>
  <Teleport to="body">
    <transition name="modal">
      <div v-if="state !== 'recording' && state !== 'paused'" class="modal-backdrop" @click="$emit('cancel')">
        <div class="modal-content" @click.stop>
          <div class="modal-header">
            <h3>Task Configuration</h3>
            <button class="close-btn" @click="$emit('cancel')" title="Close">×</button>
          </div>

          <div class="modal-body">
            <form @submit.prevent="handleSubmit">
              <div class="form-group">
                <label>Task ID *</label>
                <input
                  v-model="form.task_id"
                  type="text"
                  required
                  placeholder="e.g., task_123"
                />
              </div>

              <div class="form-group">
                <label>Device ID *</label>
                <input
                  v-model="form.device_id"
                  type="text"
                  required
                  placeholder="e.g., robot_01"
                />
              </div>

              <div class="form-group">
                <label>Data Collector ID</label>
                <input
                  v-model="form.data_collector_id"
                  type="text"
                  placeholder="e.g., collector_01"
                />
              </div>

              <div class="form-group">
                <label>Scene</label>
                <input
                  v-model="form.scene"
                  type="text"
                  placeholder="e.g., warehouse_navigation"
                />
              </div>

              <div class="form-group">
                <label>Subscene</label>
                <input
                  v-model="form.subscene"
                  type="text"
                  placeholder="e.g., aisle_traversal"
                />
              </div>

              <div class="form-group">
                <label>Factory</label>
                <input
                  v-model="form.factory"
                  type="text"
                  placeholder="e.g., factory_shenzhen"
                />
              </div>

              <div class="form-group">
                <label>Operator Name</label>
                <input
                  v-model="form.operator_name"
                  type="text"
                  placeholder="e.g., operator_001"
                />
              </div>

              <div class="form-group">
                <label>Skills (comma-separated)</label>
                <input
                  v-model="skillsText"
                  type="text"
                  placeholder="e.g., navigation, obstacle_avoidance"
                />
              </div>

              <div class="form-group">
                <label>Topics (comma-separated)</label>
                <input
                  v-model="topicsText"
                  type="text"
                  placeholder="e.g., /camera/image, /lidar/scan, /odom"
                />
              </div>

              <div class="form-group">
                <label>Start Callback URL</label>
                <input
                  v-model="form.start_callback_url"
                  type="url"
                  placeholder="http://server.example.com/api/start"
                />
              </div>

              <div class="form-group">
                <label>Finish Callback URL</label>
                <input
                  v-model="form.finish_callback_url"
                  type="url"
                  placeholder="http://server.example.com/api/finish"
                />
              </div>

              <div class="form-group">
                <label>User Token (JWT)</label>
                <textarea
                  v-model="form.user_token"
                  rows="2"
                  placeholder="eyJhbGciOiJIUzI1NiIs..."
                ></textarea>
              </div>
            </form>
          </div>

          <div class="modal-footer">
            <button type="button" @click="$emit('cancel')" class="cancel-btn">
              Cancel
            </button>
            <button type="submit" form="config-form" class="submit-btn" @click="handleSubmit">
              ✓ Set Config
            </button>
          </div>
        </div>
      </div>
    </transition>
  </Teleport>
</template>

<script setup>
import { reactive, computed, watch } from 'vue'

const props = defineProps({
  state: String
})

const emit = defineEmits(['submit', 'cancel'])

const form = reactive({
  task_id: `task_${Date.now()}`,
  device_id: 'robot_01',
  data_collector_id: 'collector_01',
  scene: 'warehouse_navigation',
  subscene: 'aisle_traversal',
  factory: 'factory_shenzhen',
  operator_name: 'operator_001',
  skills: ['navigation', 'obstacle_avoidance'],
  topics: ['/camera/image', '/lidar/scan', '/odom'],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
})

const skillsText = computed({
  get() {
    return form.skills.join(', ')
  },
  set(value) {
    form.skills = value.split(',').map(s => s.trim()).filter(s => s)
  }
})

const topicsText = computed({
  get() {
    return form.topics.join(', ')
  },
  set(value) {
    form.topics = value.split(',').map(s => s.trim()).filter(s => s)
  }
})

function handleSubmit() {
  emit('submit', { ...form })
}
</script>

<style scoped>
.modal-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 2000;
  padding: 1rem;
}

.modal-content {
  background: white;
  border-radius: 12px;
  box-shadow: 0 20px 60px rgba(0,0,0,0.3);
  max-width: 600px;
  width: 100%;
  max-height: 90vh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.modal-header {
  background: #4f46e5;
  color: white;
  padding: 1rem 1.5rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(255,255,255,0.1);
}

.modal-header h3 {
  margin: 0;
  font-size: 1.125rem;
  font-weight: 600;
}

.close-btn {
  background: rgba(255, 255, 255, 0.2);
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
  min-width: 44px;
  min-height: 44px;
  user-select: none;
  -webkit-tap-highlight-color: transparent;
}

.close-btn:hover {
  background: rgba(255, 255, 255, 0.3);
}

.close-btn:active {
  background: rgba(255, 255, 255, 0.25);
}

.modal-body {
  padding: 1.5rem;
  overflow-y: auto;
  flex: 1;
}

.form-group {
  margin-bottom: 1rem;
}

label {
  display: block;
  font-weight: 500;
  color: #374151;
  margin-bottom: 0.5rem;
  font-size: 0.875rem;
}

input, textarea {
  width: 100%;
  padding: 0.625rem 0.75rem;
  border: 1px solid #d1d5db;
  border-radius: 6px;
  font-size: 0.875rem;
  font-family: inherit;
  transition: border-color 0.2s;
  box-sizing: border-box;
}

input:focus, textarea:focus {
  outline: none;
  border-color: #3b82f6;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
}

.modal-footer {
  padding: 1rem 1.5rem;
  border-top: 1px solid #e5e7eb;
  display: flex;
  gap: 1rem;
  justify-content: flex-end;
  background: #f9fafb;
}

button {
  padding: 0.75rem 1.5rem;
  border: none;
  border-radius: 6px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
  min-height: 44px;
  user-select: none;
  -webkit-tap-highlight-color: transparent;
}

.submit-btn {
  background: #4f46e5;
  color: white;
}

.submit-btn:hover {
  background: #2563eb;
}

.cancel-btn {
  background: #f3f4f6;
  color: #374151;
}

.cancel-btn:hover {
  background: #e5e7eb;
}

/* Modal transitions */
.modal-enter-active,
.modal-leave-active {
  transition: all 0.3s ease;
}

.modal-enter-from,
.modal-leave-to {
  opacity: 0;
}

.modal-enter-from .modal-content,
.modal-leave-to .modal-content {
  transform: scale(0.95);
}

/* Mobile responsive */
@media (max-width: 768px) {
  .modal-backdrop {
    padding: 0.5rem;
  }

  .modal-content {
    max-height: 95vh;
    border-radius: 8px;
  }

  .modal-header {
    padding: 1rem;
  }

  .modal-header h3 {
    font-size: 1rem;
  }

  .modal-body {
    padding: 1rem;
  }

  .modal-footer {
    padding: 0.75rem 1rem;
    flex-direction: column-reverse;
    gap: 0.5rem;
  }

  label {
    font-size: 0.85rem;
  }

  input, textarea {
    padding: 0.75rem;
    font-size: 16px; /* Prevents iOS zoom on focus */
  }

  button {
    width: 100%;
    padding: 0.875rem;
    font-size: 1rem;
  }
}
</style>
