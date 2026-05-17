#!python3
import json, sys
from shutil import which
from statistics import median
from subprocess import run
from timeit import timeit

if not(len(sys.argv) == 3 and
       which("edge264_test-gcc") and
       which("edge264_test-clang") and
       which("ffmpeg") and
       which("avcdec") and
       which("h264dec")):
	print(f"Usage: {sys.argv[0]} <video.264> <nb_runs>\n" +
		"PATH must contain the paths to edge264_test-gcc, edge264_test-clang, ffmpeg,\n" +
		"avcdec and h264dec")
	exit(1)
with open("test.cfg", "w") as f:
	f.write(f"--input {sys.argv[1]}\n--num_cores 1")

edge264_clang, edge264_gcc, ffmpeg, libavc, openh264 = [], [], [], [], []
for _ in range(int(sys.argv[2])):
	edge264_clang.append(float(run(["edge264_test-clang", "-by", sys.argv[1]], capture_output=True).stdout.split(b'\n')[-3][5:10]))
	edge264_gcc.append(float(run(["edge264_test-gcc", "-by", sys.argv[1]], capture_output=True).stdout.split(b'\n')[-3][5:10]))
	ffmpeg.append(float(run(["ffmpeg", "-hide_banner", "-benchmark", "-threads", "1", "-c:v", "h264", "-i", sys.argv[1], "-f", "null", "-"], capture_output=True).stderr.split(b'\n')[-3][13:18]))
	libavc.append(timeit(lambda: run("avcdec", capture_output=True), number=1))
	openh264.append(float(run(["h264dec", sys.argv[1]], capture_output=True).stderr.split(b'\n')[6].split()[2]))
print(f'{{"edge264-Clang":{median(edge264_clang):.1f},"edge264-GCC":{median(edge264_gcc):.1f},"FFmpeg":{median(ffmpeg):.1f},"LibAVC":{median(libavc):.1f},"OpenH264":{median(openh264):.1f}}}')
