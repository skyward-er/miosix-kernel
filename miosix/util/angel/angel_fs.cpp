/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "angel_fs.h"

#include <sys/fcntl.h>
#include <errno.h>

#include "filesystem/stringpart.h"
#include "angel.h"

namespace miosix {

angel::Mode flagsToMode(int flags) {
    // Try to map from flags to angel mode

    // First, are we appending? This has high priority
    if(flags & _FAPPEND)
        // If yes, do we need to read?
        return flags & _FREAD ? angel::MODE_APB : angel::MODE_AB;
    
    // Second, do we need to both read and write?
    if(flags & _FREAD && flags & _FWRITE)
        // If yes, do we need to truncate the file?
        return flags & _FTRUNC ? angel::MODE_WPB : angel::MODE_RPB;

    // Finally, do we need to write?
    return angel::MODE_WB;
}

class AngelFile : public FileBase {
public:
    AngelFile(intrusive_ref_ptr<FilesystemBase> parent, int handle);
    ~AngelFile();

    ssize_t write(const void *data, size_t len) override;
    ssize_t read(void *data, size_t len) override;
    off_t lseek(off_t pos, int whence) override;
    int fstat(struct stat *pstat) const override;
    int ioctl(int cmd, void *arg) override;

private:
    int offset;
    int handle;
};

AngelFile::AngelFile(intrusive_ref_ptr<FilesystemBase> parent, int handle) : 
    FileBase(parent), 
    offset(0), 
    handle(handle) {}

AngelFile::~AngelFile() {
    angel::sys_close(handle);
}

ssize_t AngelFile::write(const void *data, size_t len) {
    // TODO: Any way to detect errors?
    int count = len - angel::sys_write(handle, data, len);
    offset += count;

    return count;
}

ssize_t AngelFile::read(void *data, size_t len) {
    // TODO: Any way to detect errors?
    int count = len - angel::sys_read(handle, data, len);
    offset += count;

    return count;
}

off_t AngelFile::lseek(off_t pos, int whence) {
    int size = angel::sys_flen(handle);
    if(size < 0)
        return -EIO;

    off_t new_offset;
    switch(whence)
    {
        case SEEK_CUR:
            new_offset = offset + pos;
            break;
        case SEEK_SET:
            new_offset = pos;
            break;
        case SEEK_END:
            new_offset = size - pos;
            break;
        default:
            return -EINVAL;
    }

    if(new_offset < 0 || new_offset > size)
        return -EOVERFLOW;

    if(angel::sys_seek(handle, new_offset) < 0) {
        return -angel::sys_errno();
    } else {
        return offset = new_offset;
    }
}

int AngelFile::fstat(struct stat *pstat) const {
    memset(pstat, 0, sizeof(struct stat));

    // TODO: Maybe actually return this error?
    int size = angel::sys_flen(handle);
    if(size < 0)
        return -EIO;

    pstat->st_dev = getParent()->getFsId();
    pstat->st_mode = S_IFREG | 0755; //-rwxr-xr-x
    pstat->st_nlink = 1;
    pstat->st_size = size;
    pstat->st_blocks = (size + 511) / 512;

    // This number needs to be very high because angel writes are very inefficient
    pstat->st_blksize = 512;

    return 0;
}

int AngelFile::ioctl(int cmd, void *arg) {
    return -EINVAL;
}

AngelFs::AngelFs() {}

int AngelFs::open(intrusive_ref_ptr<FileBase>& file, StringPart &name, int flags, int mode) {
    int handle = angel::sys_open(name.c_str(), flagsToMode(flags));
    if(handle == -1) {
        return -angel::sys_errno();
    }

    file = intrusive_ref_ptr<AngelFile>(new AngelFile(shared_from_this(), handle));
    return 0;
}

int AngelFs::lstat(StringPart& name, struct stat *pstat) {
    intrusive_ref_ptr<FileBase> file;
    int ret = open(file, name, 0, 0);
    if(ret != 0)
        return ret;

    // We only really support files
    return file.get()->fstat(pstat);
}

int AngelFs::unlink(StringPart& name) {
    if(angel::sys_remove(name.c_str()) == 0) {
        return 0;
    } else {
        return -angel::sys_errno();
    }
}

int AngelFs::rename(StringPart& oldName, StringPart& newName) {
    if(angel::sys_rename(oldName.c_str(), newName.c_str()) == 0) {
        return 0;
    } else {
        return -angel::sys_errno();
    }
}

int AngelFs::mkdir(StringPart& name, int mode) {
    // TODO: Maybe support these using angel::sys_system?
    return -EACCES;
}

int AngelFs::rmdir(StringPart& name) {
    // TODO: Maybe support these using angel::sys_system?
    return -EACCES;
}

}