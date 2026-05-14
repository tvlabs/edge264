#!python3
import json, sys
from shutil import which
from statistics import median
from subprocess import Popen, PIPE

if not(len(sys.argv) == 3 and which("edge264_test") and which("ffmpeg") and which("h264dec")):
	print(f"Usage: {sys.argv[0]} <video.264> <nb_runs>\n" +
		"PATH should point to ffmpeg, h264dec and edge264_test binaries")
	exit(1)
edge264, ffmpeg, openh264 = [], [], []
for _ in range(int(sys.argv[2])):
	edge264.append(float(Popen(f"edge264_test -by {sys.argv[1]}", shell=True, stdout=PIPE).stdout.readlines()[-2][5:10]))
	ffmpeg.append(float(Popen(f"ffmpeg -hide_banner -benchmark -threads 1 -i {sys.argv[1]} -f null -", shell=True, stdout=PIPE, stderr=PIPE).stderr.readlines()[-2][13:18]))
	openh264.append(float(Popen(f"h264dec {sys.argv[1]}", shell=True, stdout=PIPE, stderr=PIPE).stderr.readlines()[6].split()[2]))
print(json.dumps({"edge264": median(edge264), "FFmpeg": median(ffmpeg), "OpenH264": median(openh264)}))
