// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS__VISIBILITY_CONTROL_H_
#define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_EXPORT __attribute__ ((dllexport))
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_EXPORT __declspec(dllexport)
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_BUILDING_DLL
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC \
  ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_EXPORT
  #else
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC \
  ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_IMPORT
  #endif
  #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC_TYPE \
  ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC
  #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_LOCAL
#else
  #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_EXPORT __attribute__ ((visibility("default")))
  #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_IMPORT
  #if __GNUC__ >= 4
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC __attribute__ ((visibility("default")))
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC
    #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_LOCAL
  #endif
  #define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROMEA_ROBOT_TO_HUMAN_LOCALISATION_RTLS__VISIBILITY_CONTROL_H_
