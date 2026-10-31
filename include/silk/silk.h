#ifndef SILK_SILK_H
#define SILK_SILK_H

#ifdef __cplusplus
extern "C" {
#endif

#define SL_VERSION_MAJOR 0
#define SL_VERSION_MINOR 1
#define SL_VERSION_PATCH 0

/* Encoded engine version: (major << 16) | (minor << 8) | patch. */
int sl_version(void);

#ifdef __cplusplus
}
#endif

#endif /* SILK_SILK_H */
