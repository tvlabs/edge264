#!/usr/bin/env python3

# Output a description of frames dependencies in DOT format to second argument,
# Then convert to svg using GraphViz: dot -Tsvg output.dot > output.svg

from os import path
import sys
from types import SimpleNamespace
import yaml

def map_dicts(list_of_dicts):
	return map(lambda d: SimpleNamespace(**d), list_of_dicts)

def main():
	graph = {}
	with open(sys.argv[1], "r") as f:
		nals = yaml.load(f, Loader=yaml.CLoader)
		for nal in map_dicts(nals):
			if "RefPicLists" not in vars(nal) or nal.FrameId > 20: continue
			for RefPicList in nal.RefPicLists:
				for ref in RefPicList:
					graph.setdefault(nal.FrameId, set()).add(ref)
	with open(sys.argv[2], "w") as f:
		print("digraph dependencies {", file=f)
		for dst, srcs in graph.items():
			for src in srcs:
				print(f"\t{src} -> {dst};", file=f)
		print("}", file=f)

if __name__ == "__main__":
	main()