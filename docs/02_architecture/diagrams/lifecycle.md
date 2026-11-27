# Lifecycle states

The hardware component (the driver, `positronic_hardware`) and the safety supervisor
(`positronic_safety`) are both lifecycle objects. Each arrow is a transition and what triggers it.

## Hardware component

`supervised.launch.py` starts it **inactive**: the port is open and the pose is published, but
nothing moves until the supervisor activates it.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Unconfigured
    Unconfigured --> Inactive: configure<br/>open the port, start the serial thread
    Inactive --> Active: activate (supervisor or launch)<br/>checks heartbeat, reply, limits, temperature<br/>then power the servos on
    Active --> Inactive: deactivate (supervisor halt)<br/>send stop, wait for the arm to confirm it
    Inactive --> Unconfigured: cleanup<br/>close the port
    Active --> Unconfigured: error (refused command, fault, failed transition)<br/>on_error sends a stop
    Inactive --> Unconfigured: error
```

## Safety supervisor

Any trigger runs the same halt, then the supervisor deactivates itself.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Unconfigured
    Unconfigured --> Inactive: configure<br/>subscriptions, ~/stop, clients
    Inactive --> Active: activate (a person)<br/>refused during a halt, without a mission heartbeat<br/>or without the controller manager<br/>resume: hardware active, then the controllers
    Active --> Halting: trigger: heartbeat lost, driver fault, fault data lost,<br/>~/stop, deactivated by a person, resume failed
    state "Halting (still ACTIVE, not a lifecycle state)" as Halting {
        direction LR
        [*] --> HardwareInactive
        HardwareInactive --> GoalsCancelled: cancel arm_controller goals
        GoalsCancelled --> ControllersOff: switch the command controllers off
        ControllersOff --> [*]
    }
    Halting --> Inactive: deactivates itself
    Inactive --> Unconfigured: cleanup
```
