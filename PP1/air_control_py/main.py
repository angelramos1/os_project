import ctypes
import mmap
import os
import signal
import subprocess
import threading
import time

_libc = ctypes.CDLL(None, use_errno=True)

TOTAL_TAKEOFFS = 20
STRIPS = 5
shm_data = []

# Size of shared memory for 3 integers (current process pid, radio, ground) use ctypes.sizeof()
SHM_LENGTH = ctypes.sizeof(ctypes.c_int) * 3

# Global variables and locks
planes = 0  # planes waiting
takeoffs = 0  # local takeoffs (per thread)
total_takeoffs = 0  # total takeoffs

# synchronization locks
state_lock = threading.Lock()
runway1_lock = threading.Lock()
runway2_lock = threading.Lock()



def create_shared_memory():
    """Create shared memory segment for PID exchange"""
    # 1. Generate shared memory name 
    shm_name = f"/shm_pids_{os.getpid()}"
    shm_name_bytes = shm_name.encode('utf-8')
    
    # 2. Temporarily adjust the permission mask
    old_mask = os.umask(0)
    
    # 3. Use _libc.shm_open to create the shared memory
    O_CREAT = 0o100
    O_RDWR = 0o2
    fd = _libc.shm_open(shm_name_bytes, O_CREAT | O_RDWR, 0o666)
    if fd == -1:
        errno = ctypes.get_errno()
        raise OSError(errno, f"shm_open failed: {os.strerror(errno)}")
    
    # 4. Set the size of the shared memory
    if _libc.ftruncate(fd, SHM_LENGTH) == -1:
        errno = ctypes.get_errno()
        _libc.close(fd)
        _libc.shm_unlink(shm_name_bytes)
        raise OSError(errno, f"ftruncate failed: {os.strerror(errno)}")
    
    # 5. Restore the original permission mask
    os.umask(old_mask)
    
    # 6. Map the shared memory
    memory = mmap.mmap(fd, SHM_LENGTH)
    
    # 7. Create an integer-array view
    data = memoryview(memory).cast('i')  # 'i' for int
    
    # 8. Return the file descriptor, mmap object and memory view
    return fd, memory, data



def HandleUSR2(signum, frame):
    """Handle external signal indicating arrival of 5 new planes.
    Complete function to update waiting planes"""
    global planes
    # increment the global variable planes
    planes += 5


def TakeOffFunction(agent_id: int):
    """Function executed by each THREAD to control takeoffs.
    Complete using runway1_lock and runway2_lock and state_lock to synchronize"""
    global planes, takeoffs, total_takeoffs

    while True:
        # Check termination condition
        with state_lock:
            if total_takeoffs >= TOTAL_TAKEOFFS:
                break

        # Try to acquire one of the two runways
        got_runway = False
        used_runway = None
        
        if runway1_lock.acquire(blocking=False):
            got_runway = True
            used_runway = runway1_lock
        elif runway2_lock.acquire(blocking=False):
            got_runway = True
            used_runway = runway2_lock
        else:
            # No runway available, wait a short time and retry
            time.sleep(0.001)
            continue

        try:
            # We have a runway, try to perform a takeoff if any planes waiting
            with state_lock:
                if planes > 0 and total_takeoffs < TOTAL_TAKEOFFS:
                    planes -= 1
                    takeoffs += 1
                    total_takeoffs += 1

                    # If we've reached a block of 5 takeoffs notify radio
                    if takeoffs == 5:
                        # radio pid is stored in shm_data[1]
                        if len(shm_data) > 1 and shm_data[1] > 0:
                            os.kill(shm_data[1], signal.SIGUSR1)
                        takeoffs = 0

            # Simulate takeoff time
            time.sleep(1)

        finally:
            # Release the runway
            if used_runway:
                used_runway.release()

    # If we reach here, check if we should notify radio to terminate
    if len(shm_data) > 1 and shm_data[1] > 0:
        os.kill(shm_data[1], signal.SIGTERM)


def launch_radio():
    """unblock the SIGUSR2 signal so the child receives it"""
    def _unblock_sigusr2():
        signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGUSR2})

    # Launch the external 'radio' process using subprocess.Popen()
    shm_name = f"/shm_pids_{os.getpid()}"
    process = subprocess.Popen(['../radio/radio', shm_name], preexec_fn=_unblock_sigusr2)
    return process


def main():
    global shm_data

    # set the handler for the SIGUSR2 signal to HandleUSR2
    signal.signal(signal.SIGUSR2, HandleUSR2)
    
    # Create the shared memory and store the current process PID using create_shared_memory()
    fd, memory, data = create_shared_memory()
    shm_data = data
    
    # Store our PID in the first position
    data[0] = os.getpid()
    
    # Run radio and store its PID in shared memory, use the launch_radio function
    radio_process = launch_radio()
    data[1] = radio_process.pid
    
    # Create and start takeoff controller threads (STRIPS) 
    threads = []
    for i in range(STRIPS):
        thread = threading.Thread(target=TakeOffFunction, args=(i,))
        threads.append(thread)
        thread.start()
    
    # Wait for all threads to finish their work
    for thread in threads:
        thread.join()
    
    # Wait for radio process to finish
    radio_process.wait()
    
    # Release shared memory and close resources
    shm_data = None  # Clear reference to memory view
    memory.close()
    os.close(fd)



if __name__ == "__main__":
    main()