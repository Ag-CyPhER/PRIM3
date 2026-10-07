# Dataset and Results Folder Structure

<details>
  <summary><h1>1. Plant-level Dataset</h1></summary>

There are 24 plots (samples) in this dataset and are split into `calibration` (4 plots) and `evaluation` (20 plots). `output/unseen` holds predictions for 260 further plots whose inputs (images, sparse model, depth maps) are not part of this release yet; they will be released soon separately.

## Folder structure

```         
plant_level/
  dataset/                       inputs: the 24 plots
    calibration/<PLOT>/          IMG_4274, IMG_4364, IMG_4458, IMG_4503
    evaluation/<PLOT>/           the other 20 plots (two of these are excluded from Yield assessment, due to inaccuracies)
      images/                    <PLOT>_NNNN.png        original file names
      sparse/                    cameras.txt  images.txt  points3D.txt   (COLMAP Sparse Reconstruction)
      depth/depth_maps.npz       one (H, W) float32 depth array per frame
  output/                        PRIM3 pipeline predictions
    calibration/<PLOT>/          4 plots
    evaluation/<PLOT>/           20 plots
    unseen/<PLOT>/               260 plots (no inputs in this release)
      centroids_<PLOT>.ply       predicted pod centroids
      pods_labeled_<PLOT>.ply    predicted pod point clouds, one label per pod
  ground_truth/
    gt_centroids_<PLOT>.ply      GT pod centroids for 24 plots
  scene_rgb_pointcloud/
    <PLOT>.ply                   RGB point cloud of the scene for 24 plots
```

## Naming conventions

-   `<PLOT>` is the capture ID, `IMG_NNNN`. The same ID names the plot folder or file in every top-level
    folder, so everything that belongs to one plot is found by that ID.
-   Split folders are `calibration` and `evaluation` (in `dataset/` and `output/`) and `unseen` (in `output/` only).
-   File name patterns:

| folder                           | file name                 | one per                                  |
|----------------------------------|---------------------------|------------------------------------------|
| `dataset/<split>/<PLOT>/images/` | `<PLOT>_NNNN.png`         | frame                                    |
| `output/<split>/<PLOT>/`         | `centroids_<PLOT>.ply`    | plot (calibration, evaluation, unseen)   |
| `output/<split>/<PLOT>/`         | `pods_labeled_<PLOT>.ply` | plot (calibration, evaluation, unseen)   |
| `ground_truth/`                  | `gt_centroids_<PLOT>.ply` | plot (calibration, evaluation)           |
| `scene_rgb_pointcloud/`          | `<PLOT>.ply`              | plot (calibration, evaluation)           |



## sparse/ (COLMAP)

COLMAP Sparse Reconstruction:

-   `cameras.txt`: one camera, `PINHOLE`, params `fx fy cx cy`, 960 x 539.
-   `images.txt`: two lines per image. Line 1 is `IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME`, line 2 the 2D keypoints `X Y POINT3D_ID ...` (`-1` when the point has no 3D point in this subset).
-   `points3D.txt`: `POINT3D_ID X Y Z R G B ERROR TRACK[]`. Tracks keep only images in the subset; points left with no track are dropped.

Pose convention (standard COLMAP): world -\> camera, `X_cam = R(q) @ X_world + t`, quaternion order `w x y z`.


## depth/depth_maps.npz

One array per frame, key = the image file name.

-   shape `(H, W)`, dtype `float32`
-   value = camera-space **z**
-   `0` = no depth value
-   units = the units of the sparse model (COLMAP scene units)


Back-projection of pixel `(u, v)` with depth `z`:

```         
X_cam   = ((u - cx) * z / fx,  (v - cy) * z / fy,  z)
X_world = R.T @ (X_cam - t)
```

Depth loading Example:

``` python
import numpy as np
depth = np.load("depth/depth_maps.npz")
d = depth["IMG_4503_0041.png"]
```

## output/

Predictions of the PRIM3 pipeline. Each plot folder holds two point cloud (PLY) files and `ground_truth/`:

-   `centroids_<PLOT>.ply`: one point (`float x y z`) per predicted pod.
-   `pods_labeled_<PLOT>.ply`: the points of the predicted pods: `float x y z`, `uchar red green blue` and `int labels`. Every point carries the label of the pod it belongs to.

## ground_truth/

Ground-truth pod centroids, one file per calibration/evaluation plot: `gt_centroids_<PLOT>.ply` (PLY, `float x y z`), one point per annotated pod.

## scene_rgb_pointcloud/

One RGB point cloud of the scene per calibration or evaluation plot: `<PLOT>.ply` (`float x y z`, `float nx ny nz`, `uchar red green blue`) and `output/`, for viewing the ground truth and the predictions on the scene.



</details>


<details>
  <summary><h1>2. Plot-level Dataset</h1></summary>

The 58 samples are split into `calibration` (4 samples) and `evaluation` (54 samples). `output/unseen` holds predictions for 100 further samples whose inputs (images, sparse model, depth maps) are not part of this release yet; they will be released separately soon.

## Folder structure

```
plot_level/
  dataset/                       inputs: 58 samples
    calibration/<SAMPLE>/        IMG_3544, IMG_3635, IMG_3707, IMG_3709
    evaluation/<SAMPLE>/         the other 54 samples
      images/                    <SAMPLE>_NNNN.png        original file names
      sparse/                    cameras.txt  images.txt  points3D.txt   (COLMAP Sparse model)
      depth/depth_maps.npz       one (H, W) float32 depth array per frame
  output/                        PRIM3 pipeline predictions
    calibration/<SAMPLE>/        4 samples
    evaluation/<SAMPLE>/         54 samples
    unseen/<SAMPLE>/             100 samples (no inputs or ground truth in this release)
      centroids_<SAMPLE>.ply     predicted pod centroids
      pods_labeled_<SAMPLE>.ply  predicted pod point clouds, one label per pod
  ground_truth/
    gt_centroids_<SAMPLE>.ply    GT pod centroids for 58 samples
  scene_rgb_pointcloud/
    <SAMPLE>.ply                 RGB point cloud of the scene (the 58 calibration and evaluation samples)
```


## Naming conventions

-   `<SAMPLE>` is the capture ID, `IMG_NNNN`. The same ID names the sample folder or file in every top-level
    folder, so everything that belongs to one sample is found by that ID.
-   Split folders are `calibration` and `evaluation` (in `dataset/` and `output/`) and `unseen` (in `output/` only).
-   File name patterns:

| folder                             | file name                   | one per                                  |
|------------------------------------|-----------------------------|------------------------------------------|
| `dataset/<split>/<SAMPLE>/images/` | `<SAMPLE>_NNNN.png`         | frame                                    |
| `output/<split>/<SAMPLE>/`         | `centroids_<SAMPLE>.ply`    | sample (calibration, evaluation, unseen) |
| `output/<split>/<SAMPLE>/`         | `pods_labeled_<SAMPLE>.ply` | sample (calibration, evaluation, unseen) |
| `ground_truth/`                    | `gt_centroids_<SAMPLE>.ply` | sample (calibration, evaluation)         |
| `scene_rgb_pointcloud/`            | `<SAMPLE>.ply`              | sample (calibration, evaluation)         |



## sparse/ (COLMAP)

COLMAP Sparse Reconstruction:

-   `cameras.txt`: one camera, `PINHOLE`, params `fx fy cx cy`, with the image size of that sample.
-   `images.txt`: two lines per image. Line 1 is `IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME`, line 2 the 2D keypoints `X Y POINT3D_ID ...` (`-1` when the keypoint has no 3D point).
-   `points3D.txt`: `POINT3D_ID X Y Z R G B ERROR TRACK[]`.

Pose convention (standard COLMAP): world -> camera, `X_cam = R(q) @ X_world + t`, quaternion order `w x y z`.

## depth/depth_maps.npz

One array per frame, key = the image file name, e.g. `IMG_3635_0000.png`.

-   shape `(H, W)` equal to the image size of the sample, dtype `float32`
-   value = camera-space **z**
-   `0` = no depth, never NaN or inf
-   units = the units of the sparse model (COLMAP scene units)

The values are COLMAP geometric depth.

Back-projection of pixel `(u, v)` with depth `z`:

```
X_cam   = ((u - cx) * z / fx,  (v - cy) * z / fy,  z)
X_world = R.T @ (X_cam - t)
```

Depth loading Example:

``` python
import numpy as np
depth = np.load("depth/depth_maps.npz")
d = depth["IMG_3635_0000.png"]          # (H, W) float32
```

## output/

Predictions of the PRIM3 pipeline. Each sample folder holds two point cloud (PLY) files, in the same
world coordinates as the sparse model and `ground_truth/`:

-   `centroids_<SAMPLE>.ply`: one point (`float x y z`) per predicted pod.
-   `pods_labeled_<SAMPLE>.ply`: the points of the predicted pods: `float x y z`, `uchar red green blue` and
    `int labels`. Every point carries the label of the pod it belongs to; labels are pod IDs and are not
    necessarily consecutive.

## ground_truth/

Ground-truth pod centroids, one file per calibration or evaluation sample: `gt_centroids_<SAMPLE>.ply` (binary
little-endian PLY, `float x y z`), one point per annotated pod, in the same world coordinates as the sparse model.

## scene_rgb_pointcloud/

One RGB point cloud of the scene per calibration or evaluation sample: `<SAMPLE>.ply` (binary little-endian PLY,
`float x y z`, `float nx ny nz`, `uchar red green blue`), in the same world coordinates as `ground_truth/` and
`output/`, for viewing the ground truth and the predictions on the scene.


</details>