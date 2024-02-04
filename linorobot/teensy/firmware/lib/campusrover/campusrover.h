#ifndef CAMPUSROVER_H
#define CAMPUSROVER_H


#define LOG_INFO(format, ...)                                                  \
  if (false) {                                                                         \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    if (false) node_handle->loginfo(buffer);                                   \
  }

#define LOG_ERROR(format, ...)                                                 \
  if (false) {                                                                         \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    node_handle->logerror(buffer);                                             \
  }

#define LOG_DEBUG(format, ...)                                                 \
  if (false) {                                                                         \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    if (false) node_handle->logdebug(buffer);                                  \
  }

#endif
