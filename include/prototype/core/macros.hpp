#ifndef PROTOTYPE_CORE_MACROS_HPP
#define PROTOTYPE_CORE_MACROS_HPP

namespace proto {

// MACROS
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

// MACROS
#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

} //  namespace proto
#endif // PROTOTYPE_CORE_MACROS_HPP
