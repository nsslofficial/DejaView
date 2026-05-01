# Been There, Scanned That: Nostalgia-Driven Point Cloud Compression for Self-Driving Cars

This repository contains the official implementation of **DejaView**, the 3D point cloud compression system introduced in the paper *"Been There, Scanned That: Nostalgia-Driven Point Cloud Compression for Self-Driving Cars"*. It provides a comprehensive framework for compressing, decompressing, and evaluating 3D Point Cloud Data (PCD) for autonomous vehicles.

In addition to our proposed DejaView approach, this repository includes integrated evaluation scripts for comparing against several state-of-the-art baselines:
* **Draco** (Google)
* **Octree Compression** (Point Cloud Library)
* **G-PCC / TMC13** (MPEG Geometry Point Cloud Compression)

## Repository Structure

```text
DejaView/
├── bash_scripts/          # Automation scripts to run full experimental pipelines
├── dejaview/              # Source code for the proposed DejaView algorithm
├── draco/                 # Source code for Draco baseline implementation
├── lib/                   # Shared C++ libraries and utilities used across methods
├── metrics/               # Evaluation code (Chamfer Distance, PSNR, etc.)
├── misc/                  # Python utilities (e.g., PCD/PLY format conversion)
├── mpeg-pcc-tmc13-*/      # Source code for the MPEG G-PCC TMC13 baseline
└── octree/                # Source code for the PCL Octree baseline
```

---

## 1. Installation and Prerequisites

The pipeline is primarily built in C++ with select utility scripts in Python 3. It has been tested on Ubuntu Linux.

### C++ Dependencies
You must install the Point Cloud Library (PCL), OpenMP, Eigen3, Boost, and VTK. 

**Important:** We highly recommend using the default system version of PCL to avoid CMake linking conflicts. On Ubuntu 20.04/22.04, you can install the necessary packages via:
```bash
sudo apt update
sudo apt install libpcl-dev libomp-dev
```

### Python Dependencies
Python is utilized for point cloud format conversions (`.pcd` to `.ply`) required by GPCC.
```bash
pip install open3d numpy
```

---

## 2. Building the Project

The project is modular, meaning each evaluation baseline and metric tool must be compiled. 

### Compiling DejaView, Metrics, Draco, and Octree
Navigate to each respective directory (`dejaview`, `metrics`, `draco`, `octree`) and build the code using CMake. For example:
```bash
cd metrics
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../..
```
*Repeat this process for the `dejaview`, `draco`, and `octree` directories.*

### Compiling GPCC (TMC13)
To run the GPCC baseline, build the included MPEG TMC13 reference software:
```bash
cd mpeg-pcc-tmc13-release-v14.0
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../..
```

---

## 3. Dataset Preparation

**Download the dataset:**
You can download the sample point cloud dataset required to run these experiments from our Google Drive:
👉 **[Download Dataset (Google Drive) - Link Coming Soon](#)**

Once downloaded, extract the contents and place them in a `sample_data/` directory located in the root of this repository.

Ensure your raw `.pcd` sequences are structured by sequence ID (e.g., `11` or `test`). The file structure must match the following exactly:
```text
DejaView/
└── sample_data/
    └── 11/                 # Sequence ID
        └── pcds/
            ├── 1.pcd
            ├── 2.pcd
            └── ...
```
During execution, the scripts will automatically generate adjacent directories (`plys/`, `compressed/`, `decompressed/`) to store intermediate and output data without modifying your original files.

---

## 4. Running the Experiments

All automation scripts are located in the `bash_scripts/` directory. These scripts run the full pipeline: reading the data, performing compression/decompression, and calculating Chamfer Distance and PSNR error metrics.

First, navigate to the scripts directory:
```bash
cd bash_scripts
```

### Running Individual Baselines
You can execute specific compression algorithms by running their respective scripts:
```bash
bash draco.sh
bash octree.sh
bash gpcc.sh
```

### Running DejaView
To evaluate the proposed method:
```bash
bash dejaview.sh
```

### Running the Full Suite
To execute all baselines and the proposed method sequentially:
```bash
bash run_all.sh
```

### Evaluation Results
As the scripts complete, the evaluation metrics (PSNR and Chamfer Distance) will be appended to text files within the `bash_scripts/` directory (e.g., `dejaview_CD.txt`, `draco_PSNR.txt`). 

---

## Citation
If you use this code in your research, please consider citing our paper.

**Full BibTeX coming soon.**
