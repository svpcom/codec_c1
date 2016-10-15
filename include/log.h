#if 0
#include <syslog.h>
#define ALOGD(fmt, ...) syslog(LOG_DEBUG, fmt, __VA_ARGS__)
#define ALOGI(fmt, ...) syslog(LOG_INFO, fmt, __VA_ARGS__)
#define ALOGE(fmt, ...) syslog(LOG_ERR, fmt, __VA_ARGS__)
#else
#define ALOGD(fmt, ...) fprintf(stderr, fmt "\n", __VA_ARGS__)
#define ALOGI(fmt, ...) fprintf(stderr, fmt "\n", __VA_ARGS__)
#define ALOGE(fmt, ...) fprintf(stderr, fmt "\n", __VA_ARGS__)
#endif
