# gn10_motor

Motor control library for HTMDv2.2FW.
Header-only C++17 library designed for embedded systems and ROS 2 nodes.

## Features

- **Header-only**: Easy integration with CMake `INTERFACE` library.
- **Template-based**: Supports `float` and `double`.
- **No dynamic allocation**: adhering to embedded constraints.
- **Modern C++**: Using C++17 features like `std::clamp` and `static_assert`.

## Components

### PID Controller (`gn10_motor/pid.hpp`)

Standard PID controller with:
- Integral windup protection (clamping)
- Derivative on Measurement (prevents derivative kick on setpoint change)
- Output limiting

**Usage:**

```cpp
#include <gn10_motor/pid.hpp>

// 1. Configure the PID controller
gn10_motor::PIDConfig<float> pid_config;
pid_config.kp = 1.5f;
pid_config.ki = 0.05f;
pid_config.kd = 0.01f;
pid_config.integral_limit = 10.0f; // Anti-windup limit
pid_config.output_limit   = 24.0f; // Max output voltage/current/etc.

// 2. Instantiate
gn10_motor::PID<float> pid(pid_config);

// 3. Update in control loop
float dt = 0.001f; // 1ms
float command = pid.update(target_velocity, current_velocity, dt);
```

### Acceleration Limiter (`gn10_motor/acceleration_limiter.hpp`)

Slew rate limiter to smooth out setpoint changes or limit physical acceleration.

**Usage:**

```cpp
#include <gn10_motor/acceleration_limiter.hpp>

// Max acceleration: 100.0 rad/s^2
gn10_motor::AccelerationLimiter<float> limiter(100.0f);

// Update in control loop
float smooth_target = limiter.update(raw_target, dt);
```

## Integration

In your `CMakeLists.txt`:

```cmake
add_subdirectory(path/to/gn10_motor)
target_link_libraries(your_target PRIVATE gn10_motor)
```
