#include "builtin_unreachable.h"

#ifndef BUILTIN_UNREACHABLE_EXIST
#include <cassert>
void __builtin_unreachable(void){assert(false);}
#endif