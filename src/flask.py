import subprocess
import sys
from multiprocessing import shared_memory
import numpy as np
import os
import uuid
from flask import Flask, render_template, request ,jsonify
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

        sizes = [
            int.from_bytes(buffer[i:i + 4], byteorder='little')
            for i in range(0, 32, 4)
        ]

        current_offset = 32  # 4 bytes * 8 arrays for their sizes
        arrays = []

        for size in sizes:
            array_start = current_offset
            array_end = array_start + size * 4  # Assuming float32 (4 bytes per element)
            array_data = np.frombuffer(buffer[array_start:array_end], dtype=np.float32)
            arrays.append(np.array(array_data))  # Create a copy of the array
            current_offset = array_end
        return [array.tolist() for array in arrays]

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
                    array0 , array1 , array2 , array3 , array4 , array5 , array6 , array7 = getSimulationFromMemory(str(unique_id))
                    returnData = {
                    "Xpsoition": array0,
                    "Yposition": array1,
                    "Zposition": array2,
                    "Vehicle State 0": array3,
                    "Vehicle State 1": array4,
                    "Vehicle State 2": array5,
                    "Velocity": array6,
                    "gForce": array7
                    }
                    return jsonify(returnData) , render_template("simulation.html")
                except FileNotFoundError:
                    message = "Shared memory block not found."
                except Exception as e:
                    message = f"Error retrieving simulation data: {str(e)}"
            else:
                message = f"Simulation failed: {stderr}"

        except Exception as e:
            message = f"Error: {str(e)}"

    return render_template("preseta.html", message=message, arrays=arrays)

if __name__ == "__main__":
    app.run(host="192.168.50.161", port=5000, debug=True)
                                                          