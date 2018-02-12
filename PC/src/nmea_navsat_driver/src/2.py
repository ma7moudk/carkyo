import mmap
import os
import struct
import time

def main():
    # Open the file for reading
    fd = os.open('/tmp/mmaptest', os.O_RDONLY)

    # Memory map the file
    buf = mmap.mmap(fd, mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_READ)

    i = None
    s = None
 
    while 1:
        new_i = struct.unpack('f', buf[:4])
        new_s = struct.unpack('f', buf[4:8])

        if i != new_i :
            print 'Steering : %f' % (new_i)
            i = new_i

        if s != new_s :
            print 'Speed : %f' % (new_s)
            s = new_s

if __name__ == '__main__':
    main()
