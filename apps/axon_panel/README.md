# Axon Webtool - RPC Debugger

A Vue 3-based web interface for debugging and controlling the Axon Recorder HTTP RPC API.

## Features

- **Real-time State Monitoring**: View current recorder state and task configuration
- **Recording Statistics**: Monitor messages received, written, dropped, and file size
- **Control Panel**: Execute RPC commands (config, begin, pause, resume, finish, cancel)
- **Activity Log**: Track all RPC interactions with timestamps
- **Visual State Transitions**: See available state transitions

## Prerequisites

- Node.js 18+ and npm/yarn
- Axon Recorder running with HTTP RPC server on port 8080

## Installation

```bash
cd tools/webtool
npm install
```

## Development

```bash
# Start development server (runs on port 3000)
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview
```

## Usage

1. Start the Axon Recorder with HTTP RPC enabled:
   ```bash
   axon_recorder --config config.yaml
   ```

2. In another terminal, start the webtool:
   ```bash
   cd tools/webtool
   npm run dev
   ```

3. Open your browser to `http://localhost:3000`

## RPC Endpoints

The webtool interacts with the following RPC endpoints:

- `GET /health` - Health check
- `GET /rpc/state` - Get current state
- `GET /rpc/stats` - Get recording statistics
- `POST /rpc/config` - Set task configuration
- `POST /rpc/begin` - Start recording
- `POST /rpc/finish` - Finish recording
- `POST /rpc/pause` - Pause recording
- `POST /rpc/resume` - Resume recording
- `POST /rpc/cancel` - Cancel recording
- `POST /rpc/clear` - Clear configuration
- `POST /rpc/quit` - Quit program

See [docs/designs/rpc-api-design.md](../../docs/designs/rpc-api-design.md) for detailed API documentation.

## Configuration

The webtool proxies requests to the Axon Recorder at `http://localhost:8080`. To change the backend URL:

```bash
# Set environment variable
export VITE_API_BASE_URL=http://your-server:port
npm run dev
```

## Project Structure

```
tools/webtool/
├── src/
│   ├── api/           # API client
│   │   └── rpc.js
│   ├── components/    # Vue components
│   │   ├── ConnectionStatus.vue
│   │   ├── StatePanel.vue
│   │   ├── ControlPanel.vue
│   │   ├── ConfigPanel.vue
│   │   └── LogPanel.vue
│   ├── App.vue        # Root component
│   └── main.js        # Entry point
├── index.html
├── package.json
└── vite.config.js
```

## Technology Stack

- Vue 3 - Progressive JavaScript framework
- Vite - Build tool and dev server
- Axios - HTTP client
