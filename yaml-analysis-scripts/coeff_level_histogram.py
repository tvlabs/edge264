#!/usr/bin/env python3

# Plot frequencies of IDCT coeffs per value (on CAVLC file logged with -V),
# to estimate the impact of optimizing coeff_abs_level_minus1 (9.3.2.3)

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
		counts = [0] * 16
		for nal in map_dicts(nals):
			for mb in map_dicts(vars(nal).get("macroblocks_cavlc") or []):
				for block in map_dicts(vars(mb).get("coeffLevels") or []):
					for c in map(abs, vars(block).get("c") or []):
						if c > 0:
							counts[min(c, 15)] += 1
		plt.bar(np.array([*range(16)]), np.array(counts))
		plt.xlabel("coeff value")
		plt.ylabel("# occurrences")
		plt.show()

if __name__ == "__main__":
	main()