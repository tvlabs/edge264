#include "edge264_internal.h"

// This file is compiled twice: once for assume functions and once for assert functions
#undef ASNAME
#undef ASCHECK
#ifndef ASSERT
	#define ASSERT 1
#endif
#if ASSERT
	#define ASNAME(f) assert_ ## f
	#define ASCHECK(cond) assert(cond)
#else
	#define ASNAME(f) assume_ ## f
	#define ASCHECK(cond) __CPROVER_assume(cond)
#endif


/**
 * Edge264GetBits CAVLC/CABAC configuration depends on who is using it.
 */
static void ASNAME(Edge264GetBits_CPB)(const Edge264GetBits *gb) {
	ASCHECK(gb->end != NULL);
	ASCHECK(gb->cpb_off >= 0 || (ptrdiff_t)__CPROVER_POINTER_OFFSET(gb->end) + gb->cpb_off >= 0);
}
static void ASNAME(Edge264GetBits_cavlc)(const Edge264GetBits *gb) {
	ASNAME(Edge264GetBits_CPB)(gb);
	ASCHECK(gb->lsb_cache != 0);
}
static void ASNAME(Edge264GetBits_cabac)(const Edge264GetBits *gb) {
	ASNAME(Edge264GetBits_CPB)(gb);
	ASCHECK(gb->range >= 256);
	ASCHECK(gb->offset < gb->range);
}


#undef ASSERT
