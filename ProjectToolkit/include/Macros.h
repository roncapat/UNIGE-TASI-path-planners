#ifndef RONCAPAT_GLOBAL_PLANNERS_MACROS_H
#define RONCAPAT_GLOBAL_PLANNERS_MACROS_H
extern const float SQRT2;
#define SQUARE(x) ((x)*(x))
#define CATH(x, y) std::sqrt(SQUARE((x))- SQUARE((y)))
#define HYPOT(x, y) (float)std::hypot(x,y)
#define INTERP_1(from, to, delta) ((from) + ((to)-(from))*(delta))
#define INTERP_ABS(from, to, delta) ((from) + ((to)-(from))*(delta))
#ifdef NDEBUG
#define RETURN_CHECK_POSITIVE(x) return(x)
#define RETURN_CHECK_POSITIVE_LIMITED(x) return(x)
#else
#define RETURN_CHECK_POSITIVE(x) auto _y = x; assert(_y>0); return(_y);
#define RETURN_CHECK_POSITIVE_LIMITED(x) auto _y = x; assert(_y>0 and _y<INFINITY); return(_y);
#endif
#endif //RONCAPAT_GLOBAL_PLANNERS_MACROS_H
