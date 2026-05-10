import json, base64, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# load input data
results = json.loads(open(0).read())
archs = list(results.keys())
decs = list(list(results.values())[0].keys())

# create output chart
x = np.arange(len(decs))
width = 1 / (len(archs) + 1)
fig, ax = plt.subplots(figsize=(6, 4), layout="constrained")
for i, (arch, timings) in enumerate(results.items()):
	rects = ax.bar(x + i * width, timings.values(), width - width / 8, label=arch, zorder=3)
	ax.bar_label(rects, padding=3)
ax.set_xticks(x + width * (len(decs) - 1) / 2 - width / 2, decs)
ax.set_ylabel("Seconds", color="#555", fontsize=10)
ax.set_title("Decoding time (lower is better)", color="#555")
ax.set_ylim(0, 1.1 * max(max(r.values()) for r in results.values()))
ax.tick_params(colors="#555")
ax.spines[:].set_color("#555")
ax.grid(axis="y", color="#aaa", linestyle="--", linewidth=0.7, zorder=0)
ax.legend(facecolor="#222", edgecolor="#aaa", labelcolor="#fff", fontsize=10)

# save summary in output file
plt.savefig("benchmark.png", dpi=150)
b64 = base64.b64encode(Path("benchmark.png").read_bytes()).decode()
summary = f"""
# Benchmark results

<img src="data:image/png;base64,{b64}" alt="Benchmark chart" width="700"/>

Test video: https://test-videos.co.uk/vids/bigbuckbunny/mp4/h264/1080/Big_Buck_Bunny_1080_10s_30MB.mp4
"""
with open(sys.argv[1], "a") as f:
	f.write(summary)
