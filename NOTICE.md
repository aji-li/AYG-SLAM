# Source and license notices

- The RGB-D and stereo entry points under `Examples/` derive from ORB-SLAM2
  examples, with AYG-SLAM integration changes. Upstream copyright and license
  headers are preserved or restored on the derived OctoMap entry points.
  Upstream: https://github.com/raulmur/ORB_SLAM2.
- Launch, build, environment, and dataset scripts are copied from AYG-SLAM.
  Machine-specific dataset paths have been replaced with generic placeholders.
- YAML configurations describe camera calibration and model entry points.
- `Examples/pt_to_onnx.py` adapts the original export helper to accept a model path.
- `tools/summarize_evo_multirun.py` adapts the original evaluation helper:
  sequence counts are supplied via JSON, missing sequences are skipped, and
  overlapping or existing output directories are rejected.
- `demo/create_demo_results.py` was added for this showcase. Its generated
  trajectories and errors are synthetic fixtures, not experimental evidence.
- The bilingual READMEs provide project documentation and demonstration links.

Code in this repository is distributed under GPLv3; see [License-gpl.txt](License-gpl.txt).
ORB-SLAM2-derived files retain their upstream GPLv3-or-later notices. No
third-party implementations or weights are bundled. Third-party authorship and
licenses remain with their respective projects.

## Dataset attribution

The video and previews include imagery from the
[TUM RGB-D Dataset and Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset),
sequence `freiburg3_walking_xyz`.

**J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers.**
*A Benchmark for the Evaluation of RGB-D SLAM Systems.* IROS, 2012.

The dataset is provided under
[Creative Commons Attribution 4.0](https://creativecommons.org/licenses/by/4.0/),
as stated on the dataset website. This recording adds AYG-SLAM feature,
segmentation, and map visualizations, plus explanatory titles. The previews are
resized excerpts of the recording. These additions do not imply endorsement by
the dataset authors. Dataset imagery remains subject to its attribution license.
