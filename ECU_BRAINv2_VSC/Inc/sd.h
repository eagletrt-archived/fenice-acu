#ifndef __SD_H__
#define __SD_H__
#include "stdbool.h"
#include "global_variables.h"
#include "fatfs.h"
#include "string.h"
#include "stdio.h"

void init_sd();
bool clean_sd();
void print_files_name();


#endif
