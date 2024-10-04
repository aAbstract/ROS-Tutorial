# ROS Tutorial Part 1 - Introduction

### 1.1 What is ROS
- ROS (Robot Operating System) is an open-source, flexible framework for writing robotics software

### 1.2 Why ROS
- **Modularity:** Provides necessary conventions and tools for building reusable robotics software components
- **Communication:** Provides a Pub-Sub implementation to facilitate communication between application parts
- **Community:** Large, active community contributing to a rich ecosystem of packages and libraries

### 1.3 ROS Versions
- **ROS 1:** Centralized/Master-Slave architecture which means single point of failure (ROS Master)
- **ROS 2:** Decentralized/Distributed which means more robustness to failures

### 1.4 ROS Key Concepts
- **Nodes:** Processes that perform certain tasks or computations in a robot control system (typically consists of one or more nodes)
- **Topics:** Named buses over which nodes exchange messages. Nodes can publish or subscribe to topics
- **Messages:** Data structures exchanged between nodes over topics
- **Services:** Unlike the Pub-Sub communication model where data flows in one direction using ROS Topics, ROS services are used for synchronous, bidirectional data flow where a client sends a request and waits for a response from the server
- **Actions:** ROS Actions are used for complex, long-duration tasks that need ongoing feedback and the ability to be preempted

### 1.5 ROS 1 System Diagram
1. ROS Master process must be running
2. Publisher node connects to master node
3. Subscriber node connects to master node and listens on topic /topic_name
4. Publisher sends a message over topic /topic_name
5. The message is now consumed by the subscriber node
<p align="center">
    <img src="ros architecture.png" alt="ROS 1 System Diagram" />
</p>
