#ifndef SPACEDSL_GLOBAL_H
#define SPACEDSL_GLOBAL_H

#if defined(SPACEDSL_LIBRARY)
#  define SPACEDSLSHARED_EXPORT __declspec(dllexport)
#else
#  define SPACEDSLSHARED_EXPORT __declspec(dllimport)
#endif

#endif // SPACEDSL_GLOBAL_H
