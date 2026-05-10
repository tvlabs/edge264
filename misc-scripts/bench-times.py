#!python3
import json
from subprocess import Popen, PIPE
from sys import argv, stderr

if len(argv) != 3:
	print(f"Usage: {argv[0]} <video.264> <nb_runs>", file=stderr)
	exit(1)

nbRuns = int(argv[2])
ffmpeg = min(float(Popen(f"ffmpeg -hide_banner -benchmark -threads 1 -i {argv[1]} -f null -", shell=True, stdout=PIPE, stderr=PIPE).stderr.readlines()[-2][13:18]) for _ in range(nbRuns))
openh264 = min(float(Popen(f"./openh264/h264dec {argv[1]}", shell=True, stdout=PIPE, stderr=PIPE).stderr.readlines()[6].split()[2]) for _ in range(nbRuns))
edge264 = min(float(Popen(f"./edge264_test -by {argv[1]}", shell=True, stdout=PIPE).stdout.readlines()[-2][5:10]) for _ in range(nbRuns))
print(json.dumps({"edge264": edge264, "FFmpeg": ffmpeg, "OpenH264": openh264}))
