Deficiencies found in compilers
================================

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

I use this technique in edge264 to dispatch the decoding of Intra prediction samples (see edge264_decode.c). An alternative would be to put each leaf in a single function, create a call-table, and branch to the root with tail calls (which are optimised into simple jumps). However I need to pass 8 vectors between these functions, which cannot fit in registers.

**Unsupported** by gcc while **supported** by clang.
