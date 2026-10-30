
https://user-images.githubusercontent.com/39611941/120728405-ced64280-c517-11eb-909e-5659ff284885.mp4

# ipad-lidar-stream
Stream Apple lidar data (e.g. iPad Pro / iPhone Pro) with open3d.

Please leave a star if this helps you!

Uses [record3d](https://github.com/marek-simonik/record3d) to stream lidar data, and visualizes point clouds by open3d.

## Installation

1. Clone the repository:
```bash
git clone https://github.com/kentaroy47/apple-lidar-stream.git
cd apple-lidar-stream
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Install the [Record3D app](https://record3d.app/) on your Apple device from the App Store

## Usage

1. Connect your Apple device to the computer with a USB cable
2. Launch the Record3D app on your device
3. Run the streaming script:
```bash
python ipad_stream.py
```

### Free alternative: NeRFCapture (Wi-Fi)

If you don't want to buy Record3D, you can use [NeRFCapture](https://github.com/jc211/NeRFCapture), a free and open-source (MIT) iOS app that sends RGB + LiDAR depth over Wi-Fi with [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds).

Unlike Record3D, NeRFCapture is not a continuous stream: **one frame is sent each time you tap Send** in the app, and the point cloud is updated on every tap.

1. Install [NeRFCapture](https://apps.apple.com/us/app/nerfcapture/id6446518379) on a LiDAR iPhone / iPad
2. Install CycloneDDS **0.10.x** on your computer. The app was built with an older CycloneDDS and **crashes when it sees a newer (11.x) peer**. 0.10.5 has wheels for Python 3.7 - 3.10 only:
```bash
conda create -n lidar python=3.10
conda activate lidar
pip install -r requirements.txt "cyclonedds==0.10.5"
```
3. Connect the computer and the device to the same network, and switch the app to **Online** mode
4. Run the streaming script, then tap **Send** in the app once it shows `1 Connection(s)`:
```bash
python ipad_stream.py --source nerfcapture
```

#### Troubleshooting

To check the connection without Open3D, run the receiver alone. It prints one line per received frame (resolution, depth range, intrinsics):
```bash
python nerfcapture.py
```

If the app crashes, check that `cyclonedds==0.10.5` is installed (see above).

If the app keeps showing `0 Connection(s)`:
* **Several network interfaces (VPN, Ethernet + Wi-Fi, ...)**: CycloneDDS may listen on the wrong one. Pass the computer's IP address on the network shared with the device:
```bash
python ipad_stream.py --source nerfcapture --iface 192.168.0.10
```
* **Multicast blocked**: some routers and university / corporate Wi-Fi block the multicast that DDS uses for discovery. Pass the device's IP address (Settings > Wi-Fi > (i)) to connect to it directly:
```bash
python ipad_stream.py --source nerfcapture --iface 192.168.0.10 --peer 192.168.0.20
```
* **Local Network permission (iOS)**: Settings > Privacy & Security > Local Network > NeRFCapture must be on.
* **Windows**: allow Python through Windows Defender Firewall when prompted. If your Wi-Fi is set to a *Public* network, allow it for public networks too.
* Restart the app after starting the script on the computer.

### Keyboard Controls

The application supports the following keyboard shortcuts:

* `F`: Toggle point cloud filtering
* `R`: Start/stop recording
* `S`: Save current frame
* `E`: End recording and save combined point cloud

### Processing Options

You can customize point cloud processing by modifying these parameters in the code:

**Statistical outlier removal:**
* `nb_neighbors`: Number of neighboring points to consider (default: 20)
* `std_ratio`: Standard deviation ratio (default: 2.0)
  
**Voxel downsampling:**
* `voxel_size`: Size of voxel for downsampling (default: 0.01)

**Normal estimation:**
* `radius`: Search radius (default: 0.1)
* `max_nn`: Maximum nearest neighbors (default: 30)

## Output Formats

* Individual frames: Saved as PLY files
* Recordings: Combined point clouds saved as PLY files
* All exports include:
  * 3D point coordinates
  * RGB colors
  * Normal vectors (when enabled)

## Devices
~~Since I have only iPad Pro, I cannot test this on iPhone 12Pro or 13Pro.~~

Thanks to [#2](https://github.com/kentaroy47/apple-lidar-stream/pull/2), it works on iPhone LiDARs too.
