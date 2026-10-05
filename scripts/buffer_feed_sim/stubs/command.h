#ifndef SIM_COMMAND_H
#define SIM_COMMAND_H
#include <stdint.h>
#define DECL_COMMAND(FUNC, MSG)
#define DECL_CONSTANT(NAME, VALUE)
void sim_sendf(const char *fmt, ...);
void sim_shutdown(const char *msg) __attribute__((noreturn));
#define sendf(FMT, args...) sim_sendf(FMT, ##args)
#define shutdown(msg) sim_shutdown(msg)
#define try_shutdown(msg) sim_shutdown(msg)
#endif
