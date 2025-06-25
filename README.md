<img
  src="https://capsule-render.vercel.app/api?type=waving&color=33e9ff&height=300&section=header&text=Sdv-Failover-Orchestrator&fontSize=60&fontColor=fffc33&textBg=true&textBgColor=94dff3"
  width="100%"
/>

<img src="./system_architecture.svg" width="1000"/>

## Table of Contents

- [Introduction](#introduction)
- [Build K3s for Master Node](#build-k3s-for-master-node)
- [Build K3s for Worker Node](#build-k3s-for-worker-node)

## Introduction
This repository serves as a container orchestrator for future SDV architectures. It provides an ultra-fast failover framework built on the lightweight container orchestrator K3s (compatible with NVIDIA Jetson series) and a Sensor Watchdog Timer mechanism to preserve vehicle applications with near-zero downtime and guarantee occupant safety.

To manage multiple containerized applications, we chose K3s, a lightweight Kubernetes distribution, as our orchestration tool. Each containerized application is assigned an Automotive Safety Integrity Level (ASIL) and deployed on a distributed cluster forming a zonal architecture. Although K3s’s built-in failover can migrate containers when a node fails, its response time alone is insufficient for vehicle systems. Therefore, we propose a failover mechanism that combines a sensor-triggered watchdog with system redundancy. In experiments across various failure scenarios, our system reduced K3s’s default failover delay of over 300 seconds to under 1 second meeting the fault tolerant timing requirements for vehicle safety and demonstrating that container technology can be applied to safety critical automotive systems. We expect this architecture to make a significant contribution to future automotive E/E architecture design.


<br>


## Build K3s for Master Node

<br>

### Clone K3s Source and Checkout Desired Version
To perform a custom build of K3s, you must first download the source code from the official GitHub repository and switch to the version you want to modify or build.

```bash
git clone https://github.com/k3s-io/k3s.git

cd k3s

git checkout tags/v1.27.11+k3s1
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
### Modify version.sh
⚠️ K3s must be built with a Go version compatible with the selected K3s release.  
Using the latest Go version may cause build failures due to incompatibility.  
To ensure a successful build, install and use the Go version required by the specific K3s release.

```bash
- VERSION_GOLANG="go"$(curl -sL "${DEPENDENCIES_URL}" | yq e '.dependencies[] | select(.name == "golang: upstream version").version' -)
+ VERSION_GOLANG="go1.21.7"
```

### Customizing Core Kubernetes Code
K3s vendors in core Kubernetes modules (k8s.io/...), so if you need to patch Kubernetes itself, choose one of these methods:

**Use go.mod replace (Recommended)**

Fork the kubernetes/kubernetes repo on GitHub and apply your changes.

In your K3s project’s go.mod, add a replace directive:

```bash
require (
  k8s.io/kubernetes v1.27.11
  …
)
replace k8s.io/kubernetes => github.com/<your-user>/kubernetes v1.27.11
```

### Build K3s
```bash
go mod tidy

make download

make generate

SKIP_VALIDATE=true make
```

### Set Environment Variable to Skip Binary Download
During the build or manual installation process, K3s may attempt to download its binary from the internet.
To prevent this (e.g., when using a locally built binary), you can set the following environment variable in your shell configuration

```bash
export INSTALL_K3S_SKIP_DOWNLOAD=true

//After editing .bashrc, apply the changes with:
source ~/.bashrc
```

### Install Custom K3s by binary file
```bash
sudo cp ./bin/k3s /usr/local/bin/k3s

sudo -E INSTALL_K3S_EXEC="server --v=2 --docker --node-ip=192.168.2.100 --flannel-iface=eth0" \
        ./install.sh
```

server
Launches K3s in server (control-plane) mode, hosting the API server, controller manager, and scheduler.

* v=2
Sets the log verbosity to level 2, enabling more detailed debug output (useful for troubleshooting).

* docker
Tells K3s to use Docker as the container runtime instead of its default (containerd).

* node-ip=192.168.2.100
Forces the node’s advertised IP address to 192.168.2.100 (important when the host has multiple interfaces).

* flannel-iface=eth0
Specifies that Flannel should bind to the eth0 interface for pod networking (ensuring VXLAN/UDP traffic flows over the correct NIC).


### Add K3s Configuration
```bash
export KUBECONFIG=/home/avees/k3s.yaml
source ~/.bashrc
```

### ✅ Verify K3s Installation on the Master Node
After running the installation script, use the following commands to verify that the K3s server is running correctly:

```bash
# Check if the k3s service is active and running
sudo systemctl status k3s
```
This command shows whether the k3s systemd service is running properly.
Look for Active: active (running) to confirm successful startup.

<br>

```bash
# View node status and network details
sudo kubectl get nodes -o wide
```
This command checks whether the master node has successfully joined the cluster and is in the Ready state.
It also displays network information, such as internal/external IP addresses and container runtime.

## Build K3s for Worker Node
Kubernetes and K3s are composed of a master node and one or more worker nodes.
The steps above complete the installation of K3s on the master node.
To form the same Kubernetes cluster across the worker nodes, K3s must also be installed on each worker node.
The following section describes how to install K3s on a worker node and connect it to the master node.

### 📁 Copy Kubeconfig to Worker Node
1. To enable kubectl access on the worker node, copy the K3s kubeconfig file from the master node and set the environment variable:
```bash
# On the master node, copy the kubeconfig file to the worker node:
scp /etc/rancher/k3s/k3s.yaml user@<worker-node-ip>:/home/user/k3s.yaml
```

2. On the worker node, set the KUBECONFIG environment variable in your shell configuration
```bash
export KUBECONFIG=/home/user/k3s.yaml
```

3. Modify Server Ip Address:
```bash
    - server: https://192.0.0.1:6443
    + server: https://192.168.2.100:6443 # Master Node Ip
```

5. Apply the changes:
```bash
source ~/.bashrc
```
### 📁 Transfer K3s Binacy file to Worker Node
Copy the custom-built K3s binary from the master node to the worker node:
```bash
scp k3s/bin/k3s user@<worker-node-ip>:/usr/local/bin
```

### 🔑 Why is node-token required?
The node-token file, located at /var/lib/rancher/k3s/server/node-token on the master node, is required to securely join worker nodes to the cluster.
It acts as an authentication token, allowing the master to verify and accept the joining node.

1. Verify node-token on Master Node
```bash
sudo cat /var/lib/rancher/k3s/server/node-token

# e.g. K10696f88f6a4282fad2ff3942d63ad753ee8225ef493e08a7eff113322da6e49be::server:8908eba3e2e7496426964909aa9d7175
```

2. Install K3s on Worker Node
```bash
export INSTALL_K3S_SKIP_DOWNLOAD=true
source ~/.bashrc
```

```bash
INSTALL_K3S_SKIP_DOWNLOAD=true curl -sfL https://get.k3s.io | K3S_URL=https://192.168.2.100:6443 \
K3S_TOKEN=K10696f88f6a4282fad2ff3942d63ad753ee8225ef493e08a7eff113322da6e49be::server:8908eba3e2e7496426964909aa9d7175 \
sh -s - --docker --node-name=worker-node2 --node-ip=192.168.2.102
```

#### Explanation of each component:

* INSTALL_K3S_SKIP_DOWNLOAD=true: Prevents the script from downloading the official K3s binary; uses the custom one already installed.

* K3S_URL=https://192.168.2.100:6443: Specifies the master node’s API server address.

* K3S_TOKEN=...: Authentication token obtained from the master node (/var/lib/rancher/k3s/server/node-token).

* docker: Use Docker as the container runtime.

* node-name: Sets the name of this worker node.

* node-ip: Specifies the IP address of this worker node.

Make sure you have already copied the custom k3s binary to /usr/local/bin/k3s before running this command.

### Verify K3s Install Success
After installation, check that the K3s agent is running properly on the worker node:
```bash
sudo systemctl status k3s-agent
```
This command verifies that the K3s agent service is active and running on the worker node.

<br>

On the master node, run the following to confirm that the worker node has successfully joined the cluster:
```bash
sudo kubectl get nodes -o wide
```
This lists all nodes in the cluster along with their roles, statuses, internal IPs, and container runtimes.
The newly joined worker node should appear in the list with a Ready status.
