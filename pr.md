## Summary

Add Vue 3 web control panel (axon_panel) for browser-based recorder monitoring and control, implement mock plugin with C ABI interface for independent testing, and add placeholder apps for axon_config and axon_transfer.

## Motivation

The recorder needs a user-friendly web interface for monitoring state, controlling recording operations, and viewing activity logs. Additionally, a mock plugin enables comprehensive E2E testing without requiring a full ROS environment, improving CI/CD reliability and development velocity.

## Changes

### Web Control Panel ([apps/axon_panel/](apps/axon_panel/))
- **Vue 3 SPA** with Vite build system for fast development
- Real-time state monitoring with visual state machine diagram (Vue Flow)
- Recording control: config, begin, pause, resume, finish, cancel operations
- Activity log panel with color-coded messages and timestamps
- Connection status indicator with auto-reconnect
- Responsive design supporting desktop and mobile viewports

### Mock Plugin ([middlewares/mock/src/mock_plugin/](middlewares/mock/src/mock_plugin/))
- Implements the Axon plugin C ABI interface ([plugin_loader.hpp](apps/axon_recorder/plugin_loader.hpp))
- Supports publish/subscribe with simulated message generation
- Independent E2E testing without ROS dependencies
- Test scripts: [test_e2e_with_mock.sh](middlewares/mock/test_e2e_with_mock.sh), [test_full_workflow.sh](middlewares/mock/test_full_workflow.sh)

### HTTP RPC API Enhancements ([apps/axon_recorder/http_server.cpp](apps/axon_recorder/http_server.cpp#L548))
- `/rpc/status` endpoint now returns cached task configuration
- Exposes task_id, device_id, scene, topics, and other metadata
- Sensitive fields (callback URLs, tokens) excluded for security

### Application Structure Updates
- Moved `axon_panel` from `tools/` to `apps/` for better organization
- Added placeholder apps: [axon_config](apps/axon_config/) (CLI tool), [axon_transfer](apps/axon_transfer/) (S3 daemon)
- Removed obsolete plugin example tests

### Documentation
- New: [docs/designs/frontend-design.md](docs/designs/frontend-design.md) - Complete panel architecture
- Updated: [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) - API specification
- Updated: [CLAUDE.md](CLAUDE.md) - Project overview and build commands

## Testing

- [x] Mock plugin unit tests ([test_mock_plugin_load.cpp](middlewares/mock/src/mock_plugin/test/test_mock_plugin_load.cpp))
- [x] Mock plugin E2E tests ([test_mock_plugin_e2e.cpp](middlewares/mock/src/mock_plugin/test/test_mock_plugin_e2e.cpp))
- [x] Web panel development server (`npm run dev` at http://localhost:5173)
- [x] Panel production build (`npm run build`)

## Backward Compatibility

- Fully backward compatible - no breaking changes to existing APIs or interfaces
- New `/rpc/status` response fields are additive
- Mock plugin is optional; existing ROS plugins unchanged

## Related Files

**Added:**
- [apps/axon_panel/](apps/axon_panel/) - Vue 3 web control panel (17 files)
- [middlewares/mock/src/mock_plugin/](middlewares/mock/src/mock_plugin/) - Mock plugin with tests (5 files)
- [apps/axon_config/](apps/axon_config/) - Placeholder app
- [apps/axon_transfer/](apps/axon_transfer/) - Placeholder app
- [docs/designs/frontend-design.md](docs/designs/frontend-design.md) - Frontend architecture doc

**Modified:**
- [apps/axon_recorder/http_server.cpp](apps/axon_recorder/http_server.cpp#L548-L565) - Task config in status response
- [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) - API documentation updates
- [CLAUDE.md](CLAUDE.md) - Project structure and build commands
- [Makefile](Makefile) - Added panel build targets

**Deleted:**
- [apps/plugin_example/](apps/plugin_example/) - Replaced by mock_plugin
- [apps/axon_recorder/test/e2e/README.md](apps/axon_recorder/test/e2e/README.md) - Moved to mock plugin
