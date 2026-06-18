#pragma once

#define ESP_LOGCONFIG(tag, format, ...) \
  do {                                  \
    (void) (tag);                       \
    (void) (format);                    \
  } while (0)

#define ESP_LOGW(tag, format, ...) \
  do {                             \
    (void) (tag);                  \
    (void) (format);               \
  } while (0)
