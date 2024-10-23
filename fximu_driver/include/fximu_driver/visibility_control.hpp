#ifndef SERIAL_DRIVER__VISIBILITY_CONTROL_HPP_
#define SERIAL_DRIVER__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(SERIAL_DRIVER_BUILDING_DLL) || defined(SERIAL_DRIVER_EXPORTS)
    #define SERIAL_DRIVER_PUBLIC __declspec(dllexport)
    #define SERIAL_DRIVER_LOCAL
  #else
    #define SERIAL_DRIVER_PUBLIC __declspec(dllimport)
    #define SERIAL_DRIVER_LOCAL
  #endif
#elif defined(__linux__)
  #define SERIAL_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define SERIAL_DRIVER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SERIAL_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define SERIAL_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif