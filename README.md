# Sdv-Failover-Orchestrator

This repository documents the process of setting up a high-speed failover-optimized Kubernetes cluster on a fleet of NVIDIA Jetson Orin devices by custom-building K3s and integrating a Sensor Watchdog Timer mechanism.  

By leveraging K3s—a lightweight container orchestrator designed for future Software-Defined Vehicle (SDV) architectures—and applying a Sensor Watchdog Timer, this architecture preserves vehicle applications with near-zero downtime and guarantees occupant safety through ultra-fast failover.  

Unlike the standard K3s installation, our setup compiles K3s from source to enable ARM-specific optimizations, support additional features, and apply custom patches for Jetson Orin. It is particularly useful for robotics, edge AI, and autonomous vehicle projects where lightweight orchestration and fine-grained system control are essential.


### Clone K3s Source and Checkout Desired Version
To perform a custom build of K3s, you must first download the source code from the official GitHub repository and switch to the version you want to modify or build.

```bash
git clone https://github.com/k3s-io/k3s.git

cd k3s

git checkout tags/v1.27.11+k3s1
```

### Set Environment Variable to Skip Binary Download
During the build or manual installation process, K3s may attempt to download its binary from the internet.
To prevent this (e.g., when using a locally built binary), you can set the following environment variable in your shell configuration

```bash
export INSTALL_K3S_SKIP_DOWNLOAD=true

//After editing .bashrc, apply the changes with:
source ~/.bashrc
```

### 🔍 Check Required Go Version

To build K3s, you need a compatible Go version. Check the `go.mod` file in the source root directory to see which version is specified:

```bash
# Change to the K3s source directory
cd k3s

# Check the required Go version in go.mod
grep "^go " go.mod
```

### Install Go
```bash
wget https://go.dev/dl/go1.21.7.linux-arm64.tar.gz

sudo tar -C /usr/local -xzf go1.21.7.linux-arm64.tar.gz

export PATH=$PATH:/usr/local/go/bin
```
