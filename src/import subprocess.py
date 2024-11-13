import subprocess
import sys
import os

def run_cpp_executable(exe_path, args=None, timeout=None):

    if not os.path.isfile(exe_path):
        raise FileNotFoundError(f"Executable not found: {exe_path}")
    command = [exe_path]
    if args:
        command.extend(args)

    try:
        # Run the process
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True  # Return strings instead of bytes
        )

        # Wait for process to complete and get output
        stdout, stderr = process.communicate(timeout=timeout)
        return_code = process.returncode

        return return_code, stdout, stderr

    except subprocess.TimeoutExpired:
        process.kill()
        raise TimeoutError(f"Process timed out after {timeout} seconds")
    except Exception as e:
        raise RuntimeError(f"Error running executable: {str(e)}")



if __name__ == "__main__":
    try:
        return_code, stdout, stderr = run_cpp_executable("src/main.exe")
        if return_code == 0:
            print("Program output:", stdout)
        else:
            print("Program error:", stderr)




    except Exception as e:
        print(f"Error: {e}")