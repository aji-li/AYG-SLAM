# Source and license notices

This repository contains selected source excerpts from AYG-SLAM and small
adaptations for a public portfolio. It does not contain the full SLAM system.

- `examples/rgbd_tum_octomap.cc` is derived from the ORB-SLAM2 RGB-D example,
  with AYG-SLAM integration changes. The upstream copyright and license header
  is preserved. Upstream: https://github.com/raulmur/ORB_SLAM2.
- `examples/ros2/ayg_slam.launch.py` is copied from the AYG-SLAM launch layer;
  the machine-specific dataset default has been replaced with a generic path.
- `tools/summarize_evo_multirun.py` is adapted from the AYG-SLAM evaluation
  helper. Dataset-specific pose counts are now supplied in a JSON file;
  absent sequences are skipped and output directory overlap is rejected.
- `demo/create_demo_results.py` was added for this showcase. All generated
  trajectories and errors are synthetic fixtures, not experimental evidence.

The selected code is distributed under GPLv3; see [License-gpl.txt](License-gpl.txt).
The ORB-SLAM2-derived excerpt retains its upstream GPLv3-or-later notice.
No third-party libraries or model weights are bundled. Upstream projects retain
their own authorship and licenses. Acknowledgement does not imply that all
upstream implementations are included in this public repository.
