#!python3
import base64, json, matplotlib, matplotlib.pyplot as plt, numpy as np, sys
from pathlib import Path
from subprocess import Popen, PIPE
matplotlib.use("Agg")

if len(sys.argv) != 4:
	print(f"Usage: {sys.argv[0]} <video.264> <nb_runs> <output.md>\n" +
		"PATH should point to ffmpeg, h264dec and edge264_test binaries")
	exit(1)

# compute benchmark times
nbRuns = int(sys.argv[2])
edge264 = min(float(Popen(f"edge264_test -by {sys.argv[1]}", shell=True, stdout=PIPE).stdout.readlines()[-2][5:10]) for _ in range(nbRuns))
ffmpeg = min(float(Popen(f"ffmpeg -hide_banner -benchmark -threads 1 -i {sys.argv[1]} -f null -", shell=True, stdout=PIPE, stderr=PIPE).stderr.readlines()[-2][13:18]) for _ in range(nbRuns))
openh264 = min(float(Popen(f"h264dec {sys.argv[1]}", shell=True, stdout=PIPE, stderr=PIPE).stderr.readlines()[6].split()[2]) for _ in range(nbRuns))
times = [edge264, ffmpeg, openh264]
decs = ["edge264", "FFmpeg", "OpenH264"]
N = 3

# create output chart
x = np.arange(N)
width = 0.3
fig, ax = plt.subplots(figsize=(5, 4), layout="constrained")
rects = ax.bar(x, times, width, zorder=3)
ax.bar_label(rects, padding=3)
ax.set_xticks(x, decs)
ax.set_ylabel("Seconds", color="#555", fontsize=10)
ax.set_title("Decoding time (lower is better)", color="#555")
ax.set_ylim(0, 1.1 * max(times))
ax.tick_params(colors="#555")
ax.spines[:].set_color("#555")
ax.grid(axis="y", color="#aaa", linestyle="--", linewidth=0.7, zorder=0)

# save summary in output file
plt.savefig("benchmark.png", dpi=150)
b64 = base64.b64encode(Path("benchmark.png").read_bytes()).decode()
summary = f"""
#### Benchmark times over {nbRuns} single-threaded runs of [video](https://test-videos.co.uk/vids/bigbuckbunny/mp4/h264/1080/Big_Buck_Bunny_1080_10s_30MB.mp4)

<img src="data:image/png;base64,{b64}" width="500"/>
"""
with open(sys.argv[3], "a") as f:
	f.write(summary)
