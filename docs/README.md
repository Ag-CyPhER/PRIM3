# PRIM3 Dataset and Results

The [Google drive folder](https://drive.google.com/drive/folders/18y8q1-vgHSrc7waRYEjwIZwv0EHVp9U1?usp=sharing) contains the PRIM3 datasets and results as zip files inside `plant_level/` and `plot_level/`. Unzip all files of one dataset into a single folder named `plant_level/` or `plot_level/` to get the folder structure shown below.

| File (`plant_level_…` / `plot_level_…`) | Unzips to | Contents | Size (plant / plot) |
|---|---|---|---|
| `…_dataset.zip` | `dataset/` | images, COLMAP sparse model, depth maps | 8.5 / 28.5 GB |
| `…_output.zip` | `output/` | PRIM3 predictions | 0.7 / 2.3 GB |
| `…_ground_truth.zip` | `ground_truth/` | GT pod centroids | 21 KB / 245 KB |
| `…_scene_rgb_pointcloud.zip` | `scene_rgb_pointcloud/` | RGB point cloud of each scene | 0.5 / 3.1 GB |
| `plant_level.pt`, `plot_level.pt` | – | model weights, one per dataset | – |

`output` and `ground_truth` are enough to score the predictions.

Each sample has a capture ID `<SAMPLE>` (`IMG_NNNN`). The same ID names the sample's folder or file everywhere, so everything that belongs to one sample is found by that ID. Samples are divided into three splits:

- `calibration`: samples used to find the baseline thresholds for the PRIM3 pipeline.
- `evaluation`: samples used to evaluate the PRIM3 pipeline performance against ground truth pod centroids.
- `unseen`: additional samples with predictions only. Their images, sparse models and depth maps will be released separately soon.

<details>
<summary><h2>1. Plant-level dataset</h2></summary>

- This dataset contains 24 samples: 4 `calibration` (IMG_4274, IMG_4364, IMG_4458, IMG_4503) and 20 `evaluation`. 
- Two evaluation samples, `IMG_4266` and `IMG_4289`, are not used in the yield assessment because their measured yield is not accurate.
- `output/unseen` holds predictions for 260 further samples.


```
plant_level/
  dataset/<calibration|evaluation>/<SAMPLE>/       24 samples
    images/                    <SAMPLE>_NNNN.png        original file names
    sparse/                    cameras.txt  images.txt  points3D.txt   (COLMAP sparse model)
    depth/depth_maps.npz       one (H, W) float32 depth array per frame
  output/<calibration|evaluation|unseen>/<SAMPLE>/ 4 + 20 + 260 samples
    centroids_<SAMPLE>.ply     predicted pod centroids
    pods_labeled_<SAMPLE>.ply  predicted pod point clouds, one label per pod
  ground_truth/
    gt_centroids_<SAMPLE>.ply  GT pod centroids (24 samples)
  scene_rgb_pointcloud/
    <SAMPLE>.ply               RGB point cloud of the scene (24 samples)
```

</details>

<details>
<summary><h2>2. Plot-level dataset</h2></summary>

- This dataset 58 samples: 4 `calibration` (IMG_3544, IMG_3635, IMG_3707, IMG_3709) and 54 `evaluation`. 
- `output/unseen` holds predictions for 100 further samples.

```
plot_level/
  dataset/<calibration|evaluation>/<SAMPLE>/       58 samples
    images/                    <SAMPLE>_NNNN.png        original file names
    sparse/                    cameras.txt  images.txt  points3D.txt   (COLMAP sparse model)
    depth/depth_maps.npz       one (H, W) float32 depth array per frame
  output/<calibration|evaluation|unseen>/<SAMPLE>/ 4 + 54 + 100 samples
    centroids_<SAMPLE>.ply     predicted pod centroids
    pods_labeled_<SAMPLE>.ply  predicted pod point clouds, one label per pod
  ground_truth/
    gt_centroids_<SAMPLE>.ply  GT pod centroids (58 samples)
  scene_rgb_pointcloud/
    <SAMPLE>.ply               RGB point cloud of the scene (58 samples)
```

</details>

<details>
<summary><h2>3. Data formats (both datasets)</h2></summary>

### PLY files

| File | Properties |
|---|---|
| `centroids_<SAMPLE>.ply`, `gt_centroids_<SAMPLE>.ply` | `float x y z`, one point per predicted / annotated pod |
| `pods_labeled_<SAMPLE>.ply` | `float x y z`, `uchar red green blue`, `int labels`. Every point carries the label of the pod it belongs to; labels are pod IDs. |
| `<SAMPLE>.ply` (`scene_rgb_pointcloud/`) | `float x y z`, `float nx ny nz`, `uchar red green blue` |

### sparse/

- `cameras.txt`: one camera, `PINHOLE`, params `fx fy cx cy`, with the image size of that sample.
- `images.txt`: two lines per image. Line 1 is `IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME`, line 2 the 2D keypoints `X Y POINT3D_ID ...` (`-1` when the keypoint has no 3D point).
- `points3D.txt`: `POINT3D_ID X Y Z R G B ERROR TRACK[]`.

### depth/depth_maps.npz

One array per frame, key = the image file name (e.g. `IMG_4503_0041.png`).

- shape `(H, W)` equal to the image size of the sample, dtype `float32`
- value = camera-space **z**
- `0` = no depth
- units = the COLMAP sparse reconstruction units

</details>