
https://user-images.githubusercontent.com/39611941/120728405-ced64280-c517-11eb-909e-5659ff284885.mp4

# ipad-lidar-stream
Stream Apple lidar data (e.g. iPad Pro) with open3d.

Please leave a star if this helps you!

Uses [record3d](https://github.com/marek-simonik/record3d) to stream lidar data, and visualizes point clouds by open3d.

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/apple-lidar-stream.git
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
