#ifndef MAIN_H
#define MAIN_H

#include "em7186_types.h"
#include <mbed.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mbed_objects.h"
#include "em7186.h"



extern u8   serialCommandMode;
extern u8   apSuspendMode;
extern u8   displayText;
extern u8   reportMetaData;
extern u8   fw[];
//extern u8   warmStartFile[];
extern FILE *flog;

#endif