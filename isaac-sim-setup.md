# NVIDIA Isaac Sim 5.0.0 — Headless Docker Setup on Arch Linux

**Machine**: Lenovo Legion 5 Pro — RTX 3070 Ti Laptop GPU
**OS**: Arch Linux, kernel `6.12.75-1-lts`
**Date**: 2026-06-27

---

## Overview

This documents getting `nvcr.io/nvidia/isaac-sim:5.0.0` running headlessly via Docker with WebRTC streaming on Arch Linux. The process involved significant driver troubleshooting — summarized here so you don't repeat it.

---

## What Doesn't Work (and Why)

| Attempt | Result | Reason |
|---|---|---|
| Isaac Sim 4.5.0 | Broken | Requires driver ≤565, but 565.x can't build on kernel ≥6.12 |
| Driver 565.77 on kernel 6.12 | Broken | Missing kernel APIs: `phys_to_dma`, `dma_is_direct`, `ioremap_driver_hardened_wc` |
| Isaac Sim 5.1.0-rc.19 | Segfault | RC build, unstable |
| Driver 610.x with Isaac Sim 5.0.0 | Segfault in `librtx.scenedb.plugin.so` at `carbOnPluginStartup` | Driver 610 is not in any supported branch for Isaac Sim 5.0.0 |

**Root cause**: Isaac Sim's bundled RTX libraries are built against a specific driver ABI. Driver 610 is too new — its internal interfaces changed. Driver 565 is too old for kernel 6.12. The working range is **R570–R580**.

---

## Requirements

### System
- Arch Linux with kernel 6.12+ (LTS recommended)
- NVIDIA GPU with NVENC support (RTX series)
- Docker installed

### Driver: R580 (nvidia-open-dkms 580.119.02)
Must use the R580 production branch — **not** the latest from pacman (which is 610+).

### Software
- `nvidia-container-toolkit` (for `--runtime=nvidia`)
- `isaacsim-webrtc-streaming-client` AppImage (for viewing the stream)

---

## Step 1: Kernel Pin

Pin the LTS kernel so it doesn't get bumped:

```
# /etc/pacman.conf
IgnorePkg = linux-lts linux-lts-headers
```

---

## Step 2: Downgrade NVIDIA Driver to R580

The current Arch repo has driver 610, which is too new. Downgrade to 580.119.02 from the Arch archive.

> Uses `nvidia-open-dkms` (open-source kernel modules — required for newer kernels).

```fish
set BASE https://archive.archlinux.org/packages
set pkgs
set -a pkgs $BASE/n/nvidia-open-dkms/nvidia-open-dkms-580.119.02-1-x86_64.pkg.tar.zst
set -a pkgs $BASE/n/nvidia-utils/nvidia-utils-580.119.02-1-x86_64.pkg.tar.zst
set -a pkgs $BASE/l/lib32-nvidia-utils/lib32-nvidia-utils-580.119.02-1-x86_64.pkg.tar.zst
set -a pkgs $BASE/n/nvidia-settings/nvidia-settings-580.119.02-1-x86_64.pkg.tar.zst
sudo pacman -U $pkgs
```

### Pin the driver to prevent auto-upgrade

```fish
sudo sed -i 's/IgnorePkg = linux-lts linux-lts-headers/IgnorePkg = linux-lts linux-lts-headers nvidia-open-dkms nvidia-utils lib32-nvidia-utils nvidia-settings/' /etc/pacman.conf
```

### Reboot

```fish
sudo reboot
```

### Verify

```fish
nvidia-smi
# Should show: Driver Version: 580.119.02
```

---

## Step 3: Docker Runtime Setup

Set NVIDIA as the default Docker runtime in `/etc/docker/daemon.json`:

```json
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```

Restart Docker:

```fish
sudo systemctl restart docker
```

---

## Step 4: Pull the Isaac Sim Image

```fish
docker pull nvcr.io/nvidia/isaac-sim:5.0.0
```

This is a large image (~20GB). Requires an NVIDIA NGC account and Docker login:

```fish
docker login nvcr.io
```

---

## Step 5: Run Isaac Sim (Headless Streaming)

```fish
docker run --rm --runtime=nvidia --network=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y nvcr.io/nvidia/isaac-sim:5.0.0 ./runheadless.native.sh
```

Wait for this line (takes ~3–5 minutes):
```
Isaac Sim Full Streaming App is loaded.
```

> `--network=host` is required so the WebRTC client can reach port 49100.

### Optional: Persist cache between runs (speeds up startup)

```fish
mkdir -p ~/docker/isaac-sim/cache/{kit,ov,pip,glcache,computecache}
mkdir -p ~/docker/isaac-sim/{logs,data,documents}
```

```fish
docker run --rm --runtime=nvidia --network=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw -v ~/docker/isaac-sim/documents:/root/Documents:rw nvcr.io/nvidia/isaac-sim:5.0.0 ./runheadless.native.sh
```

---

## Step 6: Connect the Streaming Client

Download the **Isaac Sim WebRTC Streaming Client** for Linux from:
```
https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html
```

Run it:

```fish
chmod +x ~/Downloads/isaacsim-webrtc-streaming-client-*.AppImage
~/Downloads/isaacsim-webrtc-streaming-client-*.AppImage
```

Connect to:
- **IP**: `127.0.0.1`
- **Port**: `49100`

---

## Alternative: Run a Standalone Python Script (no streaming)

For scripted/automated workflows without a viewer:

```fish
docker run --rm --runtime=nvidia -e ACCEPT_EULA=Y nvcr.io/nvidia/isaac-sim:5.0.0 ./python.sh /isaac-sim/standalone_examples/api/omni.isaac.core/hello_world.py
```

---

## Key Ports

| Port | Protocol | Purpose |
|---|---|---|
| 49100 | TCP | WebRTC streaming client connection |
| 47998 | UDP | WebRTC media stream |

---

## Notes

- Only **one streaming client** can connect at a time
- NVENC is required for streaming (RTX GPUs have it; A100 does not)
- The `usdrt.hydra` and `gpu.foundation` warnings in the logs are benign
- ROS2 (Humble) is bundled inside the container — no host ROS2 needed
- If 580 ever stops working, fallback is R570 (`570.153.02-1`) from the same Arch archive

---

## Supported Driver Branches (Isaac Sim 5.0.0)

Per NVIDIA Omniverse technical requirements:

| Branch | Linux Driver |
|---|---|
| R570 | 570.169 |
| R580 | 580.95.05 |
| R595 | 595.58.03 |

Driver 610+ is **not listed** and causes crashes. R595 is not in the Arch archive as of 2026-06-27.
