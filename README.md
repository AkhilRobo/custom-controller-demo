# Forward Velocity Controller (Custom ROS 2 Controller)

## Overview
`controller_new::ForwardVelocityController` is a custom ROS 2 controller built on top of the `controller_interface::ControllerInterface` base class.  
It allows you to send **velocity commands** to a single joint and have them executed by the `ros2_control` hardware interface.

Unlike the default `velocity_controllers/ForwardCommandController`, this controller is implemented from scratch, giving you full control over:
- How commands are received
- How state/command interfaces are selected
- What happens during each lifecycle transition

---

## Architecture

```mermaid
flowchart LR
    subgraph User
        A[Publish: `/forward_velocity_controller/commands`] --> B
    end

    subgraph ControllerManager
        B[ForwardVelocityController] -->|Writes Velocity| C[Hardware Interface]
    end

    C -->|Updates joint states| D[Joint State Broadcaster]
    D -->|Publishes| E["/joint_states"]
    E --> F[Robot State Publisher]
    F -->|Publishes transforms| G["/tf"]
