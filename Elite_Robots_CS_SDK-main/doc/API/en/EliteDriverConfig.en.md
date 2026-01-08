# EliteDriverConfig

## Definition

```cpp
class EliteDriverConfig {
   public:
    // IP-address under which the robot is reachable.
    std::string robot_ip;

    // EliRobot template script file that should be used to generate scripts that can be run.
    std::string script_file_path;

    // Local IP-address that the reverse_port and trajectory_port will bound.
    std::string local_ip = "";

    // If the driver should be started in headless mode.
    bool headless_mode = false;

    // The driver will offer an interface to receive the program's script on this port.
    // If the robot cannot connect to this port, `External Control` will stop immediately.
    int script_sender_port = 50002;

    // Port that will be opened by the driver to allow direct communication between the driver and the robot controller.
    int reverse_port = 50001;

    // Port used for sending trajectory points to the robot in case of trajectory forwarding.
    int trajectory_port = 50003;

    // Port used for forwarding script commands to the robot. The script commands will be executed locally on the robot.
    int script_command_port = 50004;

    // The duration of servoj motion.
    float servoj_time = 0.008;

    // Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
    float servoj_lookhead_time = 0.1;

    // Servo gain.
    int servoj_gain = 300;

    // Acceleration [rad/s^2]. The acceleration of stopj motion.
    float stopj_acc = 8;

    // When using the `writeServojQueue()` and `writeServojPoseQueue` interfaces, the number of points pre-saved in the queue before
    // starting the movement.
    int servoj_queue_pre_recv_size = 10;

    // When using the `writeServojQueue()` and `writeServojPoseQueue` interfaces, the timeout duration for the queue waiting for
    // pre-stored points. If the value is less than or equal to 0, the timeout duration will be calculated based on
    // `servoj_queue_pre_recv_size * servoj_time`.
    float servoj_queue_pre_recv_timeout = -1;

    EliteDriverConfig() = default;
    ~EliteDriverConfig() = default;
};
```

## Description

This class serves as the configuration input when constructing the `EliteDriver` class, providing parameters such as the robot's IP address and control script path.

## Parameters

- `robot_ip`
    - Type: `std::string`
    - Description: Robot IP address.

- `script_file_path`
    - Type: `std::string`
    - Description: Template for the robot control script. An executable script will be generated based on this template.

- `local_ip`
    - Type: `std::string`
    - Description: Local IP address. When left as default (empty string), `EliteDriver` will attempt to automatically detect the local IP.
    - Note: If the robot cannot connect to your local TCP server via the `socket_open()` script command, you need to manually set this parameter or check your network.

- `headless_mode`:
    - Type: `bool`
    - Description: Whether to run in headless mode。 After using this mode, there is no need to use the 'External Control' plugin. If true, the constructor will send a control script to the robot's primary port once.

- `script_sender_port`
    - Type: `int`
    - Description: Port used for sending control scripts. If this port cannot be connected, the `External Control` plugin will stop immediately.

- `reverse_port`
    - Type: `int`
    - Description: Reverse communication port. This port primarily sends control modes and partial control data.

- `trajectory_port`
    - Type: `int`
    - Description: Port for sending trajectory points.

- `script_command_port`
    - Type: `int`
    - Description: Port for sending script commands.

- `servoj_time`
    - Type: `float`
    - Description: Time interval for `servoj()` command.

- `servoj_lookhead_time`
    - Type: `float`
    - Description: Lookahead time for `servoj()` command, range [0.03, 0.2] seconds.

- `servoj_gain`
    - Type: `int`
    - Description: Servo gain.

- `stopj_acc`
    - Type: `float`
    - Description: Acceleration for stopping motion (rad/s²).

- servoj_queue_pre_recv_size
    - Type: `int`
    - Description: When using the `writeServoj()` interface with the `queue_mode` parameter set to `true`, this refers to the number of points pre-stored in the queue before motion starts. (For queue mode details, refer to the description of `queue_mode` in the [writeServoj()](./EliteDriver.en.md#control-joint-position) interface.)

- servoj_queue_pre_recv_timeout
    - Type: `float`
    - Description:When using the `writeServoj()` interface with the `queue_mode` parameter set to `true`, the timeout duration for the queue waiting for pre-stored points. If the value is less than or equal to 0, the timeout duration will be calculated based on `servoj_queue_pre_recv_size * servoj_time`.(For queue mode details, refer to the description of `queue_mode` in the [writeServoj()](./EliteDriver.en.md#control-joint-position) interface.)