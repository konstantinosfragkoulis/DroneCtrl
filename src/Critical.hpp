#ifndef CRITICAL_H
#define CRITICAL_H

extern void cleanup();

extern void signalHandler(int signum);
extern void initSignalHandlers();

#endif // CRITICAL_H