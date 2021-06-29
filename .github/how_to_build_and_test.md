### How to build and test
```bash
# Build
colcon build --event-handlers console_direct+ --packages-select node_registry

# Test
colcon test --event-handlers console_direct+ --packages-select node_registry
```
