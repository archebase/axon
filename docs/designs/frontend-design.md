# AxonPanel - Frontend Design Document

**Date:** 2025-01-21
**Status:** Implemented

## Overview

AxonPanel is a modern Vue 3-based web interface for debugging and controlling the Axon Recorder HTTP RPC API. It provides real-time monitoring, recording control, and visual state machine representation.


## Features

- **Real-time State Monitoring**: View current recorder state and task configuration
- **Recording Statistics**: Monitor messages received, written, dropped, and file size
- **Control Panel**: Execute RPC commands (config, begin, pause, resume, finish, cancel)
- **Activity Log**: Track all RPC interactions with timestamps and color-coded messages
- **Visual State Machine**: Interactive diagram showing state transitions
- **Responsive Design**: Mobile-friendly interface with touch-optimized controls
- **Auto-refresh**: Polls state and statistics every second

## Architecture

### Component Structure

```
App.vue (Root Component)
├── ConnectionStatus.vue    - Connection status and health check
├── StatePanel.vue           - Statistics and task configuration display
├── ControlPanel.vue         - Command buttons and state machine diagram
│   └── StateMachineDiagram.vue  - Visual state transition diagram
├── ConfigPanel.vue          - Modal form for task configuration
└── LogPanel.vue            - Activity log display
```

### Data Flow

```
User Action → ControlPanel
     ↓
App.vue (Command Handler)
     ↓
RPC API Client (api/rpc.js)
     ↓
Axon Recorder (HTTP Server)
     ↓
Response → App.vue (State Update)
     ↓
Component Re-render (Vue Reactivity)
```

### Directory Structure

```
tools/axon_panel/
├── src/
│   ├── api/
│   │   └── rpc.js              # Centralized API client
│   ├── components/
│   │   ├── ConnectionStatus.vue
│   │   ├── StatePanel.vue
│   │   ├── ControlPanel.vue
│   │   ├── StateMachineDiagram.vue
│   │   ├── ConfigPanel.vue
│   │   └── LogPanel.vue
│   ├── App.vue                 # Root component
│   ├── main.js                 # Entry point
│   └── edge-label-fix.css      # Vue Flow edge label fix
├── index.html                  # HTML template
├── package.json                # Dependencies
├── vite.config.js              # Vite configuration
└── README.md                   # User documentation
```

## Key Components

### 1. App.vue - Main Application Container

**File:** [tools/axon_panel/src/App.vue](../../tools/axon_panel/src/App.vue)

**Responsibilities:**
- Manages global state (current state, task config, statistics, logs)
- Handles RPC command execution and error handling
- Polls server for state updates (1-second interval)
- Manages activity log with FIFO (100 entries max)

**State Management:**
```javascript
const connected = ref(false)           // Server connection status
const health = ref(null)               // Health check data
const currentState = ref('idle')       // Current recorder state
const taskConfig = ref(null)           // Cached task configuration
const currentTaskId = ref('')          // Active task ID
const stats = ref(null)                // Recording statistics
const showConfigPanel = ref(false)     // Config modal visibility
const showLogs = ref(true)             // Log panel visibility
const logs = ref([])                   // Activity log entries
```

**Polling Strategy:**
- Automatic state refresh every 1000ms
- Starts on component mount
- Stops on component unmount
- Fetches both state and stats

**Command Flow:**
1. User clicks button → Emit command event
2. `handleCommand()` executes appropriate RPC call
3. Success → Log message + refresh state
4. Error → Log error message

### 2. ConnectionStatus.vue - Connection Indicator

**File:** [tools/axon_panel/src/components/ConnectionStatus.vue](../../tools/axon_panel/src/components/ConnectionStatus.vue)

**Features:**
- Visual connection status badge (Connected/Disconnected)
- Display recorder version
- Manual refresh button

**Health Check:**
- Endpoint: `GET /health`
- Updates: On mount + manual refresh
- Error handling: Graceful degradation

**Props:**
```javascript
{
  connected: Boolean,    // Connection status
  health: Object         // Health data { version, running, state }
}
```

**Emits:**
- `refresh` - Manual health check request

### 3. StatePanel.vue - Statistics Display

**File:** [tools/axon_panel/src/components/StatePanel.vue](../../tools/axon_panel/src/components/StatePanel.vue)

**Displays:**
- Task configuration:
  - Task ID, Device ID, Scene, Factory
  - Topics (as tags)
- Recording statistics:
  - Messages received (formatted number)
  - Messages written (formatted number)
  - Messages dropped (formatted number)
  - File size (formatted bytes)

**Formatting Functions:**
```javascript
formatNumber(1523456)      // "1,523,456"
formatBytes(2147483648)    // "2.00 GB"
```

**Update Frequency:**
- Syncs with parent's 1-second polling
- Manual refresh button

### 4. ControlPanel.vue - Command Center

**File:** [tools/axon_panel/src/components/ControlPanel.vue](../../tools/axon_panel/src/components/ControlPanel.vue)

**Features:**
- Command buttons with state-based enable/disable
- Current task ID display
- Embedded state machine diagram

**Button Logic:**
```javascript
config:  enabled when state !== 'recording' && state !== 'paused'
begin:   enabled when state === 'ready'
pause:   enabled when state === 'recording'
resume:  enabled when state === 'paused'
finish:  enabled when state === 'recording' || state === 'paused'
cancel:  enabled when state === 'recording' || state === 'paused'
clear:   enabled when state === 'ready'
quit:    always enabled
```

**Emits:**
- `command` - With command name as payload

### 5. StateMachineDiagram.vue - Visual State Machine

**File:** [tools/axon_panel/src/components/StateMachineDiagram.vue](../../tools/axon_panel/src/components/StateMachineDiagram.vue)

**Technology:** Vue Flow (flow chart library)

**Features:**
- Four-state visualization (IDLE, READY, RECORDING, PAUSED)
- Dynamic edge visibility based on current state
- Active state highlighting with color coding
- Smooth transitions between states

**State Colors:**
```
IDLE:      Gray (#9ca3af)
READY:     Green (#10b981)
RECORDING: Red (#ef4444)
PAUSED:    Yellow (#f59e0b)
```

**Edge Visibility Logic:**
```javascript
idle:     [idle→ready]
ready:    [ready→recording, ready→idle]
recording: [recording→paused, recording→idle]
paused:   [paused→recording, paused→idle]
```

**Special Implementation:**
- Forces edge labels to black (CSS workaround for Vue Flow)
- Disables panning, zooming, and node dragging
- Responsive sizing (350px height)

### 6. ConfigPanel.vue - Task Configuration Modal

**File:** [tools/axon_panel/src/components/ConfigPanel.vue](../../tools/axon_panel/src/components/ConfigPanel.vue)

**Features:**
- Modal dialog with Teleport to body
- Form validation (HTML5 required attributes)
- Pre-filled default values for quick testing
- Comma-separated array inputs (skills, topics)
- Only shown when state !== 'recording' && state !== 'paused'

**Form Fields:**
```javascript
{
  task_id: string,              // Required
  device_id: string,            // Required
  data_collector_id: string,    // Optional
  scene: string,                // Optional
  subscene: string,             // Optional
  factory: string,              // Optional
  operator_name: string,        // Optional
  skills: string[],             // Comma-separated input
  topics: string[],             // Comma-separated input
  start_callback_url: string,   // Optional (URL validation)
  finish_callback_url: string,  // Optional (URL validation)
  user_token: string            // Optional (textarea)
}
```

**Default Values:**
```javascript
task_id: `task_${Date.now()}`
device_id: 'robot_01'
data_collector_id: 'collector_01'
scene: 'warehouse_navigation'
subscene: 'aisle_traversal'
factory: 'factory_shenzhen'
operator_name: 'operator_001'
skills: ['navigation', 'obstacle_avoidance']
topics: ['/camera/image', '/lidar/scan', '/odom']
```

**Emits:**
- `submit` - With form data object
- `cancel` - Modal closed without submission

### 7. LogPanel.vue - Activity Logger

**File:** [tools/axon_panel/src/components/LogPanel.vue](../../tools/axon_panel/src/components/LogPanel.vue)

**Features:**
- Chronological log with timestamps
- Color-coded message types
- Terminal-like appearance (dark background)
- Empty state message

**Message Types:**
```javascript
{
  timestamp: string,    // HH:MM:SS format
  message: string,      // Log message
  type: string         // 'info' | 'success' | 'warning' | 'error'
}
```

**Color Scheme:**
```javascript
info:     Blue (#60a5fa)
success:  Green (#34d399)
warning:  Yellow (#fbbf24)
error:    Red (#f87171)
```

**Props:**
```javascript
{
  logs: Array    // Array of log entry objects
}
```

## RPC API Integration

**File:** [tools/axon_panel/src/api/rpc.js](../../tools/axon_panel/src/api/rpc.js)

The centralized API client provides methods for all RPC endpoints:

```javascript
// Query endpoints
rpcApi.health()              // GET /health
rpcApi.getState()            // GET /rpc/state
rpcApi.getStats()            // GET /rpc/stats

// Command endpoints
rpcApi.setConfig(config)     // POST /rpc/config
rpcApi.begin(taskId)         // POST /rpc/begin
rpcApi.finish(taskId)        // POST /rpc/finish
rpcApi.pause()               // POST /rpc/pause
rpcApi.resume()              // POST /rpc/resume
rpcApi.cancel(taskId)        // POST /rpc/cancel
rpcApi.clear()               // POST /rpc/clear
rpcApi.quit()                // POST /rpc/quit
```

**Base URL Configuration:**
```javascript
const API_BASE_URL = 'http://localhost:8080'
```

**Response Format:**
All methods return promises that resolve to the response JSON:
```javascript
{
  success: boolean,
  message: string,
  data: {
    state: string,
    task_id: string,
    // ... other fields
  }
}
```

See [RPC API Design](rpc-api-design.md) for detailed API documentation.

## State Machine

The application follows the Axon Recorder state machine:

```
IDLE ──(POST /rpc/config)──► READY ──(POST /rpc/begin)──► RECORDING ↔ PAUSED
  ▲                             │                                       │
  │                             │                                       │
  └────────(POST /rpc/finish)───┴──────────────(POST /rpc/finish)──────┘
                                │
                                ▼
                          (POST /rpc/quit)
                                │
                                ▼
                          Program Exit
```

### State Transitions

| Current State | Valid Commands | Result State |
|---------------|----------------|--------------|
| `IDLE` | config | `READY` |
| `READY` | begin | `RECORDING` |
| `READY` | clear | `IDLE` |
| `RECORDING` | pause | `PAUSED` |
| `RECORDING` | finish | `IDLE` |
| `RECORDING` | cancel | `IDLE` |
| `PAUSED` | resume | `RECORDING` |
| `PAUSED` | finish | `IDLE` |
| `PAUSED` | cancel | `IDLE` |
| Any State | quit | Program Exit |

## UI/UX Design

### Design Principles

1. **Mobile-First**: Responsive layout optimized for tablets and phones
2. **Touch-Friendly**: Minimum 44px tap targets for buttons
3. **Visual Feedback**: Color-coded states and animated transitions
4. **Real-Time Updates**: Auto-refreshing data without manual reload
5. **Error Handling**: User-friendly error messages in activity log

### Color Scheme

```
Primary:   Indigo (#4f46e5) - Header, active elements
Success:   Green (#10b981)   - Ready state, success messages
Danger:    Red (#ef4444)     - Recording state, errors, cancel action
Warning:   Yellow (#f59e0b)  - Paused state, warnings
Neutral:   Gray (#6b7280)    - Idle state, labels
```

### Responsive Breakpoints

**Desktop (> 768px):**
- Multi-column grid layout (auto-fit, minmax(350px, 1fr))
- Floating log panel (600px × 500px, fixed position)
- Full-size buttons and text
- Hover states for buttons

**Mobile (≤ 768px):**
- Single-column layout
- Full-width log panel (60vh height, fixed to bottom)
- Touch-optimized buttons (min 44px × 44px)
- Prevented pull-to-refresh: `overscroll-behavior-y: contain`
- Prevented zoom on input focus: `font-size: 16px`

### Animations

**Modal Transitions:**
```css
.modal-enter-active, .modal-leave-active {
  transition: all 0.3s ease;
}
.modal-enter-from, .modal-leave-to {
  opacity: 0;
}
.modal-enter-from .modal-content, .modal-leave-to .modal-content {
  transform: scale(0.95);
}
```

**Slide Transitions (Log Panel):**
```css
.slide-enter-active, .slide-leave-active {
  transition: all 0.3s ease;
}
.slide-enter-from, .slide-leave-to {
  opacity: 0;
  transform: translateY(20px) scale(0.95);
}
```

## Technology Stack

- **Vue 3** (v3.4+): Progressive JavaScript framework with Composition API
- **Vite** (v5.0+): Build tool and dev server with HMR
- **Vue Flow** (v1.48+): Interactive flow chart library for state machine visualization
- **Axios** (v1.6+): HTTP client for API requests

### Dependencies

```json
{
  "dependencies": {
    "@vue-flow/background": "^1.3.2",
    "@vue-flow/controls": "^1.1.3",
    "@vue-flow/core": "^1.48.1",
    "axios": "^1.6.0",
    "vue": "^3.4.0"
  },
  "devDependencies": {
    "@vitejs/plugin-vue": "^5.0.0",
    "vite": "^5.0.0"
  }
}
```

## Development

### File Naming Convention
- Components: PascalCase (e.g., `ConnectionStatus.vue`)
- Utilities: camelCase (e.g., `rpc.js`)

### Code Style
- **Vue 3 Composition API**: Uses `<script setup>` syntax
- **Reactive State**: `ref()` for primitives, `reactive()` for objects
- **Props/Emits**: Defined with `defineProps()` and `defineEmits()`
- **Scoped Styles**: Component-specific CSS with `<style scoped>`

### Development Commands

```bash
# Install dependencies
npm install

# Start development server (http://localhost:5173)
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview
```

### Browser Compatibility
- Modern browsers with ES6+ support
- Chrome/Edge 90+, Firefox 88+, Safari 14+
- Mobile browsers (iOS Safari 14+, Chrome Android)

## Deployment

### Production Build

```bash
npm run build
```

This creates a `dist/` directory with static files.

### Nginx Configuration Example

```nginx
server {
    listen 80;
    server_name axon-panel.example.com;
    root /path/to/axon_panel/dist;

    location / {
        try_files $uri $uri/ /index.html;
    }

    # Proxy API requests to Axon Recorder
    location /rpc/ {
        proxy_pass http://localhost:8080;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }
}
```

### Docker Deployment

```dockerfile
FROM node:18-alpine as builder
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
```

## Troubleshooting

### Common Issues

**Problem**: "Failed to fetch" errors
- **Solution**: Ensure Axon Recorder is running on port 8080
- **Check**: CORS is enabled on the server

**Problem**: State not updating
- **Solution**: Check browser console for errors
- **Check**: Network tab in DevTools for failed requests

**Problem**: Buttons disabled unexpectedly
- **Solution**: Verify current state matches expected state
- **Check**: Activity log for error messages

**Problem**: Edge labels not visible (Vue Flow)
- **Solution**: Already fixed in [edge-label-fix.css](../../tools/axon_panel/src/edge-label-fix.css)
- **Workaround**: Forces all edge label text to black

## Future Enhancements

- [ ] WebSocket support for real-time updates (replace polling)
- [ ] Dark mode theme
- [ ] User preferences (API URL, refresh interval)
- [ ] Export logs to file
- [ ] Historical recording session browser
- [ ] Multiple recorder support
- [ ] Authentication integration (JWT/OAuth)
- [ ] Graph/chart visualization for statistics over time

## Related Documentation

- [RPC API Design](rpc-api-design.md) - Complete API reference
- [tools/axon_panel/README.md](../../tools/axon_panel/README.md) - User guide
