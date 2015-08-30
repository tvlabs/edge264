Deficiencies found in compilers
================================

Global Register Variables
-------------------------

Bypassing the register allocator to keep the context pointer in a single register. Though reluctant to use it initially, this has proven to be extremely efficient. Both gcc and clang had the nasty tendency to spill this pointer to stack in almost every function, though it is constantly in use. This also removes the first argument from *every single function* in `edge264_cabac.c`. As a virtuous side-effect, it also made me pack some previously-split structures into this single context, reducing the number of live pointers and parameters even further.

**Supported** by gcc (though it forbids ebp sometimes), **unsupported** by clang. This is the sole reason for gcc outperforming clang in my benchmarks :)

Support for nested switches
---------------------------

This technique is used to create a tree-like branch structure, where several code paths share common tail portions.

```c
switch (i) {
case 0 .. 1:
	switch (i) {
	case 0: break;
	case 1: break;
	}
	// Portion shared by 0 and 1
	break;
case 2 .. 4:
	switch (i) {
	case 2 .. 3:
		switch (i) {
		case 2: break;
		case 3: break;
		}
		// Portion shared by 2 and 3
		break;
	case 4: break;
	}
	// Portion shared by 2, 3 and 4
	break;
}
```

The inner switches must merge their branch tables with the outermost one, such that only the first table-jump is conditional, the rest being straight jumps down towards the tree root.

I use this technique in edge264 to dispatch the decoding of Intra prediction samples (see `edge264_intra_ssse3.c`). An alternative is to put each leaf in a single function, create a call-table, and branch to the root with tail calls (which are optimised into simple jumps), but here I need to pass 8 vectors, which would not fit in registers for any call convention.

**Unsupported** by gcc, **supported** by clang.
