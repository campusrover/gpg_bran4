#ifndef CAMPUSROVER_H
#define CAMPUSROVER_H

#define LOG_INFO(...)                                                          \
  if (true) {                                                                  \
    sprintf(buffer, __VA_ARGS__);                                              \
    node_handle->loginfo(buffer);                                              \
  }

#define LOG_ERROR(...)                                                         \
  if (true) {                                                                  \
    sprintf(buffer, __VA_ARGS__);                                              \
    node_handle->logerror(buffer);                                             \
  }

#define LOG_DEBUG(...)                                                         \
  if (true) {                                                                  \
    sprintf(buffer, __VA_ARGS__);                                              \
    node_handle->logdebug(buffer);                                             \
  }

#endif
