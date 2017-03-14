#undef MAX_PATH
#define MAX_PATH 520
#include <unistd.h>
#include <fcntl.h>

off_t filesize(int fd);
