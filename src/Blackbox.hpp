#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <string>

extern std::string getCurrentTimestamp();
extern void writeFlightDataToCSV();
extern void initializeBlackbox();
extern void closeCSV();

#endif // BLACKBOX_H