# Source and license notices

This repository contains selected AYG-SLAM source excerpts and adaptations for
public demonstration. It does not contain the complete SLAM system.

- The RGB-D and stereo entry points under `Examples/` derive from ORB-SLAM2
  examples, with AYG-SLAM integration changes. Upstream copyright and license
  headers are preserved or restored on the derived OctoMap entry points.
  Upstream: https://github.com/raulmur/ORB_SLAM2.
- Launch, build, environment, and dataset scripts are copied from AYG-SLAM.
  Machine-specific dataset paths have been replaced with generic placeholders.
- YAML configurations contain camera calibration and model entry points only.
  Unpublished algorithm settings are omitted.
- `Examples/pt_to_onnx.py` adapts the original export helper to accept a model path.
- `tools/summarize_evo_multirun.py` adapts the original evaluation helper:
  sequence counts are supplied via JSON, missing sequences are skipped, and
  overlapping or existing output directories are rejected.
- `demo/create_demo_results.py` was added for this showcase. Its generated
  trajectories and errors are synthetic fixtures, not experimental evidence.
- The main bilingual READMEs follow the development repository, with public
  scope, demo links, and links for omitted documentation adapted for this export.

Selected code is distributed under GPLv3; see [License-gpl.txt](License-gpl.txt).
ORB-SLAM2-derived files retain their upstream GPLv3-or-later notices. No
third-party implementations or weights are bundled. Third-party authorship and
licenses remain with their respective projects.

Video and preview images contain TUM RGB-D dataset imagery with AYG-SLAM overlays.
Dataset attribution and recording modifications are documented in
[docs/DEMO.md](docs/DEMO.md). The media's dataset license is separate from the
source-code license.
