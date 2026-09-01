#include "RmControlPanel.h"

#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QSurfaceFormat>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

static constexpr const char* LOCK_FILE = "/tmp/rm_control_panel.lock";

namespace {

bool setCloseOnExec(int fd)
{
    int flags = fcntl(fd, F_GETFD);
    if (flags < 0) return false;
    return fcntl(fd, F_SETFD, flags | FD_CLOEXEC) == 0;
}

void writeOwnerPid(int fd)
{
    if (ftruncate(fd, 0) != 0) {
        fprintf(stderr, "writeOwnerPid: ftruncate failed: %s\n", strerror(errno));
    }
    (void)lseek(fd, 0, SEEK_SET);
    dprintf(fd, "%ld\n", static_cast<long>(getpid()));
    fsync(fd);
}

long readOwnerPid(int fd)
{
    lseek(fd, 0, SEEK_SET);
    char buf[64] = {0};
    const ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (n <= 0) return -1;
    char* end = nullptr;
    const long pid = strtol(buf, &end, 10);
    return (end != buf && pid > 0) ? pid : -1;
}

}  // namespace

int main(int argc, char* argv[])
{
    int lockFd = open(LOCK_FILE, O_CREAT | O_RDWR | O_CLOEXEC, 0666);
    if (lockFd < 0) {
        fprintf(stderr, "Failed to open lock file %s: %s\n", LOCK_FILE, strerror(errno));
        return 1;
    }
    if (!setCloseOnExec(lockFd)) {
        fprintf(stderr, "Warning: failed to set FD_CLOEXEC on %s: %s\n", LOCK_FILE, strerror(errno));
    }
    if (flock(lockFd, LOCK_EX | LOCK_NB) < 0) {
        const long ownerPid = readOwnerPid(lockFd);
        if (ownerPid > 0) {
            fprintf(stderr, "Another rm_control_panel instance is already running (pid=%ld).\n", ownerPid);
        } else {
            fprintf(stderr, "Another rm_control_panel instance is already running.\n");
        }
        return 1;
    }
    writeOwnerPid(lockFd);

    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    QSurfaceFormat format;
    format.setOption(QSurfaceFormat::DebugContext);
    QSurfaceFormat::setDefaultFormat(format);

    RmControlPanel window;
    window.show();
    const int ret = app.exec();

    rclcpp::shutdown();
    if (ftruncate(lockFd, 0) != 0) {
        fprintf(stderr, "shutdown: ftruncate failed: %s\n", strerror(errno));
    }
    (void)flock(lockFd, LOCK_UN);
    (void)close(lockFd);
    return ret;
}
