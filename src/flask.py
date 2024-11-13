import subprocess
import sys
from multiprocessing import shared_memory
import numpy as np
import os
import uuid
from flask import Flask, render_template, request
from contextlib import contextmanager

@contextmanager
def safe_shared_memory(name):
    shm = None
    try:
        shm = shared_memory.SharedMemory(name=name)
        yield shm
    finally:
        if shm is not None:
            try:
                shm.close()
            except BufferError:
                import gc
                # Force garbage collection
                # shm.close was not properlly unlinked the memory, forced garbage collection was the only way ive found to properlly dealloc
                gc.collect()  
                try:
                    shm.close()
                except BufferError:
                    print(f"Warning: Could not close shared memory {name} immediately")
            try:
                shm.unlink()
            except FileNotFoundError:
                pass

def run_cpp_executable(exe_path, args=None, timeout=None):
    if not os.path.isfile(exe_path):
        raise FileNotFoundError(f"Executable not found: {exe_path}")

    command = [exe_path]
    if args:
        command.extend([str(arg) for arg in args])

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )

        stdout, stderr = process.communicate(timeout=timeout)
        return_code = process.returncode
       
        return return_code, stdout, stderr

    except subprocess.TimeoutExpired:
        process.kill()
        raise TimeoutError(f"Process timed out after {timeout} seconds")
    except Exception as e:
        raise RuntimeError(f"Error running executable: {str(e)}")

def getSimulationFromMemory(unique_id):
    """Get simulation data from shared memory with proper cleanup"""
    with safe_shared_memory(str(unique_id)) as shm:
        buffer = shm.buf
        # Read the lengths of the arrays
        array1_size = int.from_bytes(buffer[:4], byteorder='little')
        array2_size = int.from_bytes(buffer[4:8], byteorder='little')

        # Calculate the offsets
        array1_start = 8
        array1_end = array1_start + array1_size * 4
        array2_start = array1_end
        array2_end = array2_start + array2_size * 4

        # Create copies of the arrays to ensure they're not tied to the shared memory
        array1 = np.array(np.frombuffer(buffer[array1_start:array1_end], dtype=np.int32))
        array2 = np.array(np.frombuffer(buffer[array2_start:array2_end], dtype=np.int32))

        return array1.tolist(), array2.tolist()

app = Flask(__name__)

@app.route("/", methods=["GET", "POST"])
def index():
    message = ""
    arrays = None

    if request.method == "POST":
        unique_id = uuid.uuid4()
        print(unique_id)
        try:
            return_code, stdout, stderr = run_cpp_executable(
                "./build/main",
                args=[str(unique_id)],
                timeout=30
            )

            if return_code == 0:
                try:
                    array1, array2 = getSimulationFromMemory(str(unique_id))
                    arrays = {'array1': array1, 'array2': array2}
                    message = "Simulation completed successfully " , arrays
                except FileNotFoundError:
                    message = "Shared memory block not found."
                except Exception as e:
                    message = f"Error retrieving simulation data: {str(e)}"
            else:
                message = f"Simulation failed: {stderr}"

        except Exception as e:
            message = f"Error: {str(e)}"

    return render_template("index.html", message=message, arrays=arrays)

if __name__ == "__main__":
    # It's recommended to use a production WSGI server in production
    app.run(host="192.168.50.161", port=5000, debug=True)
                                                          