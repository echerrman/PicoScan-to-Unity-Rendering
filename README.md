# PicoScan LiDAR Unity Visualizer

A real-time point cloud visualization system for the PicoScan 150 2D LiDAR scanner, featuring live data streaming to Unity with dual rendering modes.

## Overview

This project provides a complete pipeline for receiving, parsing, and visualizing LiDAR data from PicoScan 150 sensors in Unity. It supports both persistent point accumulation for mapping and live-only mode for real-time visualization.

**Key Features:**
- Real-time UDP data reception from PicoScan 150
- Compact binary protocol parsing
- Dual visualization modes (Persistent/Live-only)
- Unity integration with instanced rendering
- Interactive UI controls for mode switching
- Point cloud downsampling and deduplication

## Requirements

### Python Dependencies
```bash
pip install numpy
```

### Unity Setup
- Unity 2019.4 LTS or newer
- Point mesh asset (Quad or Sphere)
- Instanced rendering material

### Hardware
- PicoScan 150 2D LiDAR scanner
- Network connection to scanner (UDP port 2115)

## Project Structure

```
├── listener.py          # UDP socket handling for scanner data
├── parser.py            # PicoScan protocol parser and point cloud extraction
├── unity_forwarder.py   # Main application - bridges scanner to Unity
├── PointCloudInstanced.cs # Unity script for point cloud rendering
└── README.md
```

## Quick Start

1. **Setup Python Environment**
   ```bash
   pip install numpy
   ```

2. **Configure Unity Scene**
   - Create empty GameObject
   - Attach `PointCloudInstanced.cs` script
   - Assign point mesh and material in inspector
   - Add UI buttons for mode control (optional)

3. **Run the Forwarder**
   ```bash
   python unity_forwarder.py
   ```

4. **Configure Scanner**
   - Ensure PicoScan 150 is sending data to UDP port 2115
   - Scanner should be on same network as host machine

## Usage

### Visualization Modes

- **Persistent Mode**: Accumulates all received points for mapping
- **Live-only Mode**: Shows only current frame for real-time tracking

### Controls

- **Toggle Mode**: Switch between persistent and live-only modes
- **Clear Points**: Remove all accumulated points
- **Status**: Query current mode and point counts

Send commands via UDP to port 5006:
```
PERSISTENT  # Switch to persistent mode
LIVE_ONLY   # Switch to live-only mode  
CLEAR       # Clear all points
STATUS      # Get current status
```

## Configuration

### Key Parameters

**Python (unity_forwarder.py):**
- `UNITY_PORT`: Unity receiver port (default: 5005)
- `CONTROL_PORT`: Command receiver port (default: 5006) 
- `SCANNER_PORT`: PicoScan data port (default: 2115)
- `MAX_POINTS`: Maximum accumulated points (default: 1M)

**Unity (PointCloudInstanced.cs):**
- `pointSize`: Visual size of rendered points
- `maxPoints`: Point cloud size limit
- `udpPort`: Data receiver port (must match Python)

## Network Ports

- **2115**: PicoScan 150 data transmission
- **5005**: Unity point cloud data reception  
- **5006**: Control command reception

## Troubleshooting

- Ensure firewall allows UDP traffic on required ports
- Verify PicoScan 150 network configuration matches host settings
- Check Unity console for UDP reception errors
- Monitor Python console for parsing warnings

## License

MIT License