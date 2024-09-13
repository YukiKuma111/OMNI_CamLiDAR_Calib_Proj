# PCD_Projection_2_IMG

This project provides tools for projecting 3D point cloud data onto 2D image planes, allowing you to visualize the alignment between point clouds and images. The repository includes scripts for processing single image-point cloud pairs as well as batch processing multiple files. Additionally, you can generate a video from the resulting projection images.


## Steps to Set Up

1. Git Clone:

    Navigate to your workspace and clone the repository:

    ```
    cd ~/your/workspace/
    git clone https://github.com/YukiKuma111/PCD_Projection_2_IMG.git
    ```

2. Create Input Directories:

    Run the provided script to set up the necessary directory structure:

    ```
    bash create_dir.bash
    ```

3. Input Source

    Copy your data to appropriate directory.
    For more details, please check out the __File Directory__.

    In addition, two examples of the contents of the txt file are as follows:

    - `extrinsic.txt`:
        ```txt
        extrinsic
        -0.020729  -0.999521  0.0229919  -0.0409719
        0.0420483  -0.0238481  -0.998831  -0.191893
        0.998901  -0.019738  0.0425225  0.0
        0  0  0  1
        ```

    - `intrinsic.txt`:
        ```txt
        intrinsic
        270.6563655458449 0                 325.8708451237354
        0                 268.5633688486991 230.5535168602322;
        0                 0                 1

        distortion 
        -0.3631940843607187 0.1542217925295358 0 0 0
        ```


## How to Run

1. `one_img_projection.py`

    **Functionality:**

    This script takes one single image and point cloud file as input, projects the point cloud onto the 2D image plane, and displays the result. The output can be saved as a PNG file.

    **Example Usage:**
    
    You can run the default directories as the input:

    ```bash
    python one_img_projection.py
    ```

    Or give your own directories:

    ```bash
    python one_img_projection.py -img /path/to/image.png -pcd /path/to/pointcloud.pcd -para /path/to/parameters_directory/ -s /path/to/save/projection.png
    ```

    Then press a random key to end the whole process.

    **Inputs:**
    
    - `-img`: Path to the input image file (e.g., `/path/to/image.png`).

    - `-pcd`: Path to the input point cloud file (e.g., `/path/to/pointcloud.pcd`).

    - `-para`: Directory containing the `intrinsic.txt` and `extrinsic.txt` files needed for projection.

    - `-s`: Path to save the output projection image (e.g., `~/output/projection.png`).

2. `omni_one_img_projection.py`

    **Functionality:**

    'Omni' means equirectangular image captured by omnidirectional camera. In this script, the core is to calculate azimuth and elevation angles and convert to pixel coordinates. There is no other difference between this function and the former one.

3. `auto_projection.py`

    **Functionality:**

    This script automatically processes multiple image and point cloud files from the specified directories, projects the 3D points onto the 2D image plane, and saves each projection result as an image file in the specified directory.

    **Example Usage:**

    You can run the default directories as the input:

    ```bash
    python auto_projection.py
    ```

    Or give your own directories:

    ```bash
    python auto_projection.py -img /path/to/image_directory -pcd /path/to/pcd_directory -para /path/to/parameters_directory -s /path/to/save_directory
    ```

    **Inputs:**

    - `-img`: Directory containing the input image files (e.g., `/path/to/image_directory`).

        The naming format for the image filename should be as follows: `1721366356.572147.png`

    - `-pcd`: Directory containing the input point cloud files (e.g., `/path/to/pcd_directory`).

        The naming format for the point cloud data filename should be as follows: `1724040477250738.pcd`

    - `-para`: Directory containing the `intrinsic.txt` and `extrinsic.txt` files needed for projection.

    - `-s`: Directory to save the output projection images (e.g., `/path/to/save_directory`).

4. `video_generation.bash`

    To create a video from the generated projection images, run the following script:

    ```bash
    bash video_generation.bash
    ```

## File Directory

```
PCD_Projection_2_IMG/
├── one_img_projection.py
├── auto_projection.py
├── create_dir.bash
├── video_generation.bash
├── img
│   ├── sample_img.png
│   └── ...
├── para
│   ├── extrinsic.txt
│   └── intrinsic.txt
├── pcd
│   ├── sample_pcd.pcd
│   └── ...
├── result
│   └── projection.png
└── README.md
```

## Reference

These scripts are generated by ChatGPT based on this CSDN blog:
https://blog.csdn.net/m0_51697446/article/details/138342083