#!/usr/bin/env python3

# Plot slices decoding time per size (file logged with -v[m])

import matplotlib.pyplot as plt
import numpy as np
from os import path
import sys
from types import SimpleNamespace
import yaml

def map_dicts(list_of_dicts):
	return map(lambda d: SimpleNamespace(**d), list_of_dicts)

def main():
	with open(sys.argv[1], "r") as f:
		nals = yaml.load(f, Loader=yaml.CLoader)
		x = []
		y = []
		for nal in map_dicts(nals):
			if "approx_byte_size" not in vars(nal): continue
			x.append(nal.approx_byte_size)
			y.append(nal.decoding_end_us - nal.decoding_start_us)
		plt.scatter(x, y)
		plt.xlabel("bytes")
		plt.ylabel("microseconds")
		plt.show()

if __name__ == "__main__":
	main()