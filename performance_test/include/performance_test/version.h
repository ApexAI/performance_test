#ifndef PERFORMANCE_TEST_VERSION_H
#define PERFORMANCE_TEST_VERSION_H

#ifdef PERFORMANCE_TEST_VERSION
const char* version = PERFORMANCE_TEST_VERSION;
#else
const char* version = "unknown";
#endif

#endif
