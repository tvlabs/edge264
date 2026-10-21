#include "edge264_api.c"

// cbmc edge264_api_harness.c --unwind 31 --unwinding-assertions -DEQUIVALENCE_CHECKS --function harness_find_start_code
void harness_find_start_code(void) {
	uint8_t backing[32] __attribute__((aligned(16)));
	size_t buf_off, end_off;
	__CPROVER_assume(buf_off < sizeof(backing));
	__CPROVER_assume(end_off <= sizeof(backing));
	__CPROVER_assume(buf_off <= end_off);
	const uint8_t *buf = backing + buf_off;
	const uint8_t *end = backing + end_off;
	int four_byte;
	const uint8_t *res = edge264_find_start_code(buf, end, four_byte);
	assert(res >= buf);
	assert(res <= end);
}
