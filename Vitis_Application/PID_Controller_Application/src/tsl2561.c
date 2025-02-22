/**
 * @file TSL2561.c
 * @brief Implementation file for TSL2561 luminosity sensor driver.
 *
 * This file contains the function definitions for initializing, reading data,
 * and calculating lux values from the TSL2561 sensor.
 *
 * @author Phil Nevins (p.nevins971@gmail.com)
 * @author
 * @date 2025-02-20
 *
 * @Notes
 * 20-FEB-2025  PN  Started project
 */

#include "TSL2561.h"
#include <stdio.h>
#include <stdlib.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "xil_io.h" // Needed for reading/writing AXI registers
#include "xtmrctr.h"
#include "xintc.h"
#include "nexys4IO.h"
#include "xil_types.h"
#include "xil_assert.h"
#include "xgpio.h"

// Implementation of functions will go here...

