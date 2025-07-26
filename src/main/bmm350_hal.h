#ifndef BMM350_HAL_H
#define BMM350_HAL_H

#include <stdint.h>

#include "bmm350.h"

int8_t bmm350_interface_init(struct bmm350_dev *dev);
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt);

#endif /*BMM350_HAL_H*/
