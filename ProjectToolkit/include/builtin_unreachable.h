//
// Created by patrick on 08/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_BUILTIN_UNREACHABLE_H
#define RONCAPAT_GLOBAL_PLANNERS_BUILTIN_UNREACHABLE_H

#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100  + __GNUC_PATCHLEVEL__)

#if defined __has_builtin
#if __has_builtin(__builtin_unreachable)
#define BUILTIN_UNREACHABLE_EXIST
#endif
#endif

#if GCC_VERSION >= 40500
#define BUILTIN_UNREACHABLE_EXIST
#endif

#ifndef BUILTIN_UNREACHABLE_EXIST
void __builtin_unreachable(void);
#endif

#endif //RONCAPAT_GLOBAL_PLANNERS_BUILTIN_UNREACHABLE_H
