#!/usr/bin/env python3

# Convert to Trace Event Format to second argument (file logged with -v[m])
# Then visualize it on https://ui.perfetto.dev/

from os import path
import sys
from types import SimpleNamespace
import yaml

def map_dicts(list_of_dicts):
	return map(lambda d: SimpleNamespace(**d), list_of_dicts)

def main():
	with open(sys.argv[1], "r") as f:
		nals = yaml.load(f, Loader=yaml.CLoader)
	with open(sys.argv[2], "w") as f:
		f.write("[\n")
		for nal in map_dicts(nals):
			if "thread_id" not in vars(nal): continue
			f.write(f'{{"name":"FrameId={nal.FrameId} first_mb_in_slice={nal.first_mb_in_slice}","ph":"X","ts":{nal.decoding_start_us},"dur":{nal.decoding_end_us-nal.decoding_start_us},"pid":0,"tid":{nal.thread_id}}},\n')
		f.write("]\n")

if __name__ == "__main__":
	main()