#ifndef __COMMON_HEADER__
#define __COMMON_HEADER__

typedef char int8;
typedef short int16;
typedef long int32;
typedef long long int64;

enum _VALID {
    INVALID,
    VALID
};

typedef enum _DIRECTION {
    DIRECTION_IN,
    DIRECTION_OUT
} DIRECTION;

#ifndef TRUE
#   define TRUE (1)
#endif

#ifndef FALSE
#   define FALSE (0)
#endif

#ifndef NULL
#   define NULL (void *) (0)
#endif

#ifndef ALL
#   define ALL (-1)
#endif

#ifndef ROBOT_ID_MAX
#   define ROBOT_ID_MAX 1000000
#endif

#endif
