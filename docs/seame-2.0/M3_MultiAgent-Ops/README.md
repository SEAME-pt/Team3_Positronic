# Module 3 · Distributed Multi-Agent Operations & Collaborative Scenarios

**1 March – 30 April 2027 · 8 weeks (Easter break included)**

## Introduction

One robot that works is a project. Several robots that work together, over imperfect networks, with different hardware and different teams behind them, is a system. Module 3 scales your platform from a single agent to a coordinated scenario with at least one other hardware class: a drone that scouts for a ground vehicle, a car that brings parts to an arm, a drone that guides a humanoid.

By the end of the module you run a live multi-agent scenario that keeps working while we cut a link and kill a peer.

## Motivation

Real deployments are fleets. Inspection drones report to ground crews, warehouse robots hand parcels to arms, and every one of them runs on Wi-Fi that drops packets. Three things make this hard:

- **Interfaces.** Teams that never met agree on message contracts, frames and timing, and then have to honour them.
- **Networks.** DDS discovery, QoS and bandwidth behave differently across a lossy wireless link than on a bench.
- **Failure.** A peer disappears, a link stalls, a message is late. The system must degrade safely, not stop or, worse, keep moving blindly.

Security enters here too. A fleet that accepts commands from anyone and installs any update is a liability. You will authenticate, encrypt and sign.

## What you will learn

- **Inter-platform middleware and networking.** ROS 2 DDS with explicit QoS profiles, Zenoh bridge for lossy or low-bandwidth links, namespace and domain partitioning.
- **Cybersecurity and OTA.** SROS 2 authentication and encryption, signed over-the-air updates, secure boot, threat modelling, SBOM hygiene.
- **Multi-agent integration.** Shared world model and common frame, conflict resolution, role handoff, interface contract enforcement.
- **Resilience.** Safe termination under peer loss, fault injection at the network and process level, integration tests across the whole stack.

## Reference architecture

```
   Agent A (e.g. drone + ground PC)          Agent B (e.g. car)
   +-----------------------------+           +-----------------------------+
   | mission node                |           | mission node                |
   | shared-world publisher      |  Zenoh /  | shared-world subscriber     |
   | SROS 2 identity             |<--DDS---->| SROS 2 identity             |
   | Module 1/2 stack            |  bridge   | Module 1/2 stack            |
   +-----------------------------+           +-----------------------------+
                 \                                       /
                  \--- shared world frame (TF2), task board, heartbeats ---/
```

- **Shared world frame.** One agreed map frame. Every published position is expressed in it, with a timestamp and a covariance.
- **Interface contract.** A versioned document per scenario: topics, message types, QoS, rates, frames, failure semantics. Enforced by integration tests.
- **Heartbeats and peer loss.** Each agent publishes liveness. Losing a peer triggers a defined degraded mode, never an undefined one.
- **Security.** Every agent has an SROS 2 identity; communication is authenticated and encrypted; updates are signed and verified before installation.

### Role of each platform

| Platform | Contribution to the scenario |
|---|---|
| Drone | Publishes target coordinates in the shared world frame from aerial detection |
| Car | Transports components or itself between peer-defined waypoints; intercepts targets |
| Arm | Serves docking, loading and unloading requests with visual alignment |
| Humanoid | Accepts a task handed off by a peer, executes it, reports back |

## Scenarios

Each team picks one scenario with a partner team. A bonus scenario joins the whole cohort.

| Scenario | Platforms | Summary |
|---|---|---|
| A · Perimeter guard and night surveillance | Drone + Car | Aerial thermal detection streams target coordinates over the bridge for ground interception. Signed OTA deployment in the field. |
| B · Hybrid logistics pick-and-pack | Car + Arm | The car navigates to a workstation where the arm picks items. A VLA model processes natural-language orders. State synchronized over the middleware. |
| C · Heavy mould exchange | Car + Arm | Docking sequence in simulation with lifecycle-enforced state transitions and QoS tuned for zero loss. Validated on HIL runners. |
| D · Smart waste management and sorting | Car + Arm | Container pulling and sorting. Coupling mechanism controlled by the vehicle; VLA guides visual alignment in simulation. |
| E · Search and humanoid rescue | Drone + Humanoid | The drone locates an asset in a hazardous zone; the humanoid navigates to it, retrieves it and carries it to safety. |

## Goals and deliverables

### Common to all platforms

| Deliverable | Evidence |
|---|---|
| Interface contract | Versioned document agreed and signed by both teams, with contract tests in CI |
| Live multi-agent demo | The scenario runs end to end on hardware with injected link loss and a killed peer, and ends in a safe state |
| Network report | QoS profiles chosen and why, measured latency and loss over the wireless link, bridge configuration |
| Security package | Threat model, SROS 2 configuration, a signed OTA update installed on a live agent and a tampered one rejected |
| Integration test suite | Cross-stack tests from sensors to peer messages, running in CI |
| Sprint demo and retrospective | Every two weeks |

### Per-platform demos

| Platform | Multi-agent participation | Degraded mode under peer loss |
|---|---|---|
| Drone | Publishes target coordinates in the shared frame that a peer acts on | Loss of the ground peer leads to hover or landing; loss of the link is handled as in Module 1 |
| Car | Transports between waypoints defined by a peer; intercepts a published target | Loss of the peer leads to a stop at a safe waypoint |
| Arm | Serves a docking or loading request with visual alignment | Loss of the peer aborts the sequence and returns to a home pose |
| Humanoid | Accepts and executes a handed-off task, reports completion | Loss of the peer leads to a safe crouch and a status message |
