#ifndef OSQP_CONFIGURE_H
#define OSQP_CONFIGURE_H

#if defined(_WIN32)
#  define IS_WINDOWS
#elif defined(__APPLE__)
#  define IS_MAC
#else
#  define IS_LINUX
#endif

#define OSQP_ALGEBRA_BUILTIN
#define OSQP_USE_LONG

#endif /* ifndef OSQP_CONFIGURE_H */

