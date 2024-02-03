#ifndef CAMPUSROVER_H
#define CAMPUSROVER_H


#define LOG_INFO(format, ...)                                                  \
  do {                                                                         \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    node_handle->loginfo(buffer);                                              \
  } while (0)

#endif
