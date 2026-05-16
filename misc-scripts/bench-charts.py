#!python3
import base64, datetime, json, matplotlib, matplotlib.pyplot as plt, numpy as np, sys
matplotlib.use("Agg")

# print usage if wrong number of inputs or ill-formed JSON
data = None
if len(sys.argv) == 3:
	try: data = json.loads(sys.argv[1])
	except: pass
if not data:
	print(f"Usage: {sys.argv[0]} <json> <output.svg>\n" +
		"data should be a matrix with named rows and columns encoded in JSON, like\n" +
		'{"x86":{"edge":0,"other":1},"arm64":{"edge":2,"other":3}}', file=sys.stderr)
	exit(1)
rnames = list(data.keys())
cnames = list(tuple(data.values())[0].keys())
d = datetime.datetime.today()

# generate output chart
x = np.arange(len(rnames))
width = 1 / (len(cnames) + 1)
fig, ax = plt.subplots(figsize=(6, 4), layout="constrained")
for c, cname in enumerate(cnames):
	rects = ax.bar(x + c * width, [r[cname] for r in data.values()], width * 0.9, label=cname, zorder=3)
	ax.bar_label(rects, padding=3)
ax.set_xticks(x + 0.5 - width, rnames)
ax.set_ylabel("Seconds", color="#555", fontsize=10)
ax.set_title(d.strftime("Decoding time measured on %d/%m/%Y (lower is better)"), color="#555")
ax.set_ylim(0, 1.1 * max(max(r.values()) for r in data.values()))
ax.tick_params(colors="#555")
ax.spines[:].set_color("#555")
ax.grid(axis="y", color="#aaa", linestyle="--", linewidth=0.7, zorder=0)
ax.legend(facecolor="#222", edgecolor="#aaa", labelcolor="#fff", fontsize=10)
plt.savefig(sys.argv[2])
