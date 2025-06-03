import time
import threading
import json
import sys
import psutil
import GPUtil
import os
import signal

DATA_FILE = "load_data.json"
TMP_FILE = "load_tmp.json"
PID_FILE = "load_monitor.pid"

class LoadMonitor:
    def __init__(self):
        self.cpu_loads = []
        self.gpu_loads = []
        self.running = False

    def sample_load(self):
        self.running = True
        while self.running:
            cpu = psutil.cpu_percent(interval=None)
            gpus = GPUtil.getGPUs()
            gpu_load = sum(gpu.load for gpu in gpus) / len(gpus) * 100 if gpus else 0
            self.cpu_loads.append(cpu)
            self.gpu_loads.append(gpu_load)
            time.sleep(0.5)

    def stop(self):
        self.running = False

    def save_temp(self):
        with open(TMP_FILE, "w") as f:
            json.dump({
                "cpu": self.cpu_loads,
                "gpu": self.gpu_loads,
                "timestamp": time.time()
            }, f)

def start_monitor():
    monitor = LoadMonitor()
    def handler(signum, frame):
        monitor.stop()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)

    monitor.sample_load()
    monitor.save_temp()
def stop_monitor():
    if not os.path.exists(PID_FILE):
        print("No monitoring process found.")
        return

    with open(PID_FILE, "r") as f:
        pid = int(f.read())

    try:
        os.kill(pid, signal.SIGTERM)
        print(f"Stopped monitoring process (PID: {pid})")
    except ProcessLookupError:
        print("Monitoring process not found or already terminated.")

    os.remove(PID_FILE)

    if not os.path.exists(TMP_FILE):
        print("No temporary data found.")
        return

    with open(TMP_FILE, "r") as f:
        temp_data = json.load(f)

    os.remove(TMP_FILE)

    cpu_data = temp_data["cpu"]
    gpu_data = temp_data["gpu"]
    entry = {
        "cpu_mean": sum(cpu_data) / len(cpu_data) if cpu_data else 0,
        "cpu_median": sorted(cpu_data)[len(cpu_data) // 2] if cpu_data else 0,
        "gpu_mean": sum(gpu_data) / len(gpu_data) if gpu_data else 0,
        "gpu_median": sorted(gpu_data)[len(gpu_data) // 2] if gpu_data else 0,
        "timestamp": temp_data["timestamp"]
    }

    if os.path.exists(DATA_FILE):
        try:
            with open(DATA_FILE, "r") as f:
                data = json.load(f)
                if not isinstance(data, list):
                    data = [data]
        except json.JSONDecodeError:
            data = []
    else:
        data = []

    data.append(entry)

    with open(DATA_FILE, "w") as f:
        json.dump(data, f, indent=2)

    print(json.dumps(entry, indent=2))

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ["start", "stop"]:
        print("Usage: system_load.py [start|stop]")
        sys.exit(1)

    if sys.argv[1] == "start":
        if os.path.exists(PID_FILE):
            print("Monitoring already started.")
            sys.exit(1)
        pid = os.fork()
        if pid == 0:
            start_monitor()
            sys.exit(0)
        else:
            with open(PID_FILE, "w") as f:
                f.write(str(pid))
            print(f"Started monitoring in background (PID: {pid})")

    elif sys.argv[1] == "stop":
        stop_monitor()

if __name__ == "__main__":
    main()
 