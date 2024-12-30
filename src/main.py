import subprocess
import sys
from multiprocessing import shared_memory
import numpy as np
import os
import uuid
import json
from flask import Flask, render_template, request , jsonify
from contextlib import contextmanager


@contextmanager
def safe_shared_memory(name):
    """Context manager for safely handling shared memory"""
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
                gc.collect()  # Force garbage collection
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
        current_offset = 4 * 19 # 19 arrays recieved
        sizes = [
            int.from_bytes(buffer[i:i + 4], byteorder='little')
            for i in range(0, current_offset, 4)
        ]

        arrays = []

        for size in sizes:
            array_start = current_offset
            array_end = array_start + size * 4  # Assuming float32 (4 bytes per element)
            array_data = np.frombuffer(buffer[array_start:array_end], dtype=np.float32)
            arrays.append(np.array(array_data))  # Create a copy of the array
            current_offset = array_end
        return [array.tolist() for array in arrays]



app = Flask(__name__)


@app.route("/simulation-presets")
def index():
    return render_template( "preset.html")

@app.route("/creatorCorner")
def creatorCorner():
    return render_template( "creatorCorner.html")

@app.route("/")
def landingPage():
    return render_template( "landingPage.html")

@app.route("/simulation", methods=["GET", "POST"])
def simulation():
    message = ""
    arrays = None

    if request.method == "POST":
        unique_id = uuid.uuid4()
        print(unique_id)
        try:
            dryMass = request.form.get('dryMass') 
            fuel = request.form.get('fuel') 
            lox = request.form.get('LOX') 
            fuelConsumption = request.form.get('fuelConsumption') 
            LOXConsumption = request.form.get('LOXConsumption') 
            reentryAccel = request.form.get('reentryAccel') 
            XPosition = request.form.get('Xposition')
            YPosition = request.form.get('Yposition')
            ZPosition = request.form.get('Zposition')
            XVelocity = request.form.get('Xvelocity')
            YVelocity = request.form.get('Yvelocity')
            ZVelocity = request.form.get('Zvelocity')
            XinitState = request.form.get('Xstate')
            YinitState = request.form.get('Ystate')
            ZinitState = request.form.get('Zstate')
            print("dryMass:", dryMass)
            print("lox:", lox)
            print("fuel:", fuel)
            print("LOXConsumption:", LOXConsumption)
            print("fuelConsumption:", fuelConsumption)
            print("reentryAccel:", reentryAccel)
            print("XPosition:", XPosition)
            print("YPosition:", YPosition)
            print("ZPosition:", ZPosition)
            print("XVelocity:", XVelocity)
            print("YVelocity:", YVelocity)
            print("ZVelocity:", ZVelocity)
            print("XinitState:", XinitState)
            print("YinitState:", YinitState)
            print("ZinitState:", ZinitState)


            return_code, stdout, stderr = run_cpp_executable(
                "../build/main",
                args=[str(unique_id),
                      float(dryMass),
                      float(lox),
                      float(fuel),
                      float(LOXConsumption),
                      float(fuelConsumption),
                      float(reentryAccel),
                      float(XPosition),
                      float(YPosition),
                      float(ZPosition),
                      float(XVelocity),
                      float(YVelocity),
                      float(ZVelocity),
                      float(XinitState),
                      float(YinitState),
                      float(ZinitState),],
                timeout=30
            )

            print(stdout)
            if return_code == 0:
                try:

                    array0 , array1 , array2 , array3 , array4 , array5 , array6 , array7 , array8 , array9 , array10 , array11 , array12 , array13 , array14 , array15 , array16 , array17 , array18 = getSimulationFromMemory(str(unique_id))
                    simulationData = {
                    "VectorTimeStamp": array0,
                    "Xposition": array1,
                    "Yposition": array2,
                    "Zposition": array3,
                    "vehicleState0": array4,
                    "vehicleState1": array5,
                    "vehicleState2": array6,
                    "velocity": array7,
                    "gForce": array8,
                    "gimbalAngleX": array9,
                    "gimbalAngleY": array10,
                    "mass": array11,
                    "fuel": array12,
                    "LOX": array13,
                    "VectorTimeStampReduced": array14,
                    "engineVector0": array15,
                    "engineVector1": array16,
                    "engineVector2": array17,
                    "enginePower":array18
                    }
                    return render_template("simulation.html" , simulationData = simulationData)
                except FileNotFoundError:
                    message = "Shared memory block not found."
                except Exception as e:
                    message = f"Error retrieving simulation data: {str(e)}"
            else:
                message = f"Simulation failed: {stderr}"

        except Exception as e:
            message = f"Error: {str(e)}"

    return render_template("preset.html" , message=message )

if __name__ == "__main__":
    #app.run(host="192.168.50.161", port=5000, debug=True)
    app.run()