/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>

static uint32_t data_buffer[10] = {
	0x1234,
	0x5678,
	0xaaaa,
	0xbbbb,
	0xcccc,
	0xdddd,
};

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	printf("%p\n", data_buffer);

	for (uint32_t i = 0; i < 10; i++) {
		printf("data_buffer[%d] = 0x%x\n", i, data_buffer[i]);
	}

	return 0;
}
