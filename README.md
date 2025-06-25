# sdv-failover-orchestrator

This repository documents the process of setting up a Kubernetes cluster using custom-built K3s on a cluster of NVIDIA Jetson Orin devices.

Unlike standard K3s installation, this setup compiles K3s from source to enable architecture-specific optimizations, support additional features, or apply custom patches for ARM-based Jetson boards. It is particularly useful for robotics, edge AI, or autonomous vehicle projects where lightweight orchestration and fine-grained system control are essential.

## Clone K3s Source and Checkout Desired Version
To perform a custom build of K3s, you must first download the source code from the official GitHub repository and switch to the version you want to modify or build.

```bash
git clone https://github.com/...
cd project
make
