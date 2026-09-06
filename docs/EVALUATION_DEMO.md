# 独立评估示例 / Standalone evaluation example

需要 Python 3，仅使用标准库。在仓库根目录执行：

Requires Python 3 and only its standard library. Run from the repository root:

```bash
python3 demo/create_demo_results.py demo-output
python3 tools/summarize_evo_multirun.py \
  demo-output/runs demo-output/summary \
  --expected-poses demo-output/expected_poses.json
```

示例先比较轨迹覆盖率，再分别选择 ATE 和 RPE RMSE 最小的运行。
`run_1` 应为最佳 ATE，`run_2` 应为最佳 RPE；轨迹不完整的 `run_3` 不应被选中。

The example first maximizes trajectory coverage, then independently minimizes
ATE and RPE RMSE. It selects `run_1` for ATE and `run_2` for RPE, excluding the
incomplete `run_3` despite its lower errors.

输出位于 `demo-output/summary/`，包括 CSV、Markdown 和选中的运行副本。
再次执行时请使用新的输出目录。工具读取已有指标，不计算或对齐轨迹。

Outputs in `demo-output/summary/` include CSV, Markdown, and copies of selected
runs. Use a new output directory when repeating the example. The tool reads
existing metrics; it does not calculate errors or align trajectories.

**这里生成的数值是合成测试数据，不代表 SLAM 实测精度。实际运行视频请查看
[运行与录制说明](DEMO.md)。**

**Generated values are synthetic test fixtures, not measured SLAM accuracy.
See [Run and recording notes](DEMO.md) for the actual dataset recording.**
