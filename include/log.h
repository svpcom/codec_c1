#ifdef ENCLIB_DEBUG
#define ALOGD(fmt, ...) fprintf(stderr, fmt "\n", __VA_ARGS__)
#define ALOGI(fmt, ...) fprintf(stderr, fmt "\n", __VA_ARGS__)
#define ALOGE(fmt, ...) fprintf(stderr, fmt "\n", __VA_ARGS__)
#else
#define ALOGD(fmt, ...)
#define ALOGI(fmt, ...)
#define ALOGE(fmt, ...)
#endif
