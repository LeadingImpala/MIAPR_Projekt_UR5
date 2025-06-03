import time
import threading
import json
import sys
import psutil
import GPUtil
import os

DATA_FILE = "load_data.json"

class LoadMonitor:
    def __init__(self):
        self.cpu_loads = []
        self.gpu_loads = []
        self.running = False
        self.thread = None

    def sample_load(self):
        while self.running:
            cpu = psutil.cpu_percent(interval=None)
            gpus = GPUtil.getGPUs()
            gpu_load = 0.0
            if gpus:
                gpu_load = sum(gpu.load for gpu in gpus) / len(gpus) * 100
            self.cpu_loads.append(cpu)
            self.gpu_loads.append(gpu_load)
            time.sleep(0.5)

    def start(self):
        if self.running:
            print("Already running.")
            return
        self.running = True
        self.cpu_loads = []
        self.gpu_loads = []
        self.thread = threading.Thread(target=self.sample_load)
        self.thread.start()

        self.start_timer = time.time()

    def stop(self):
        if not self.running:
            print("Not running.")
            return
        self.running = False
        self.thread.join()

        cpu_mean = sum(self.cpu_loads) / len(self.cpu_loads) if self.cpu_loads else 0
        gpu_mean = sum(self.gpu_loads) / len(self.gpu_loads) if self.gpu_loads else 0
        cpu_median = sorted(self.cpu_loads)[len(self.cpu_loads)//2] if self.cpu_loads else 0
        gpu_median = sorted(self.gpu_loads)[len(self.gpu_loads)//2] if self.gpu_loads else 0

        self.end_timer = time.time()

        time_elapsed = self.start_timer - self.end_timer

        new_entry = {
            "cpu_mean": cpu_mean,
            "cpu_median": cpu_median,
            "gpu_mean": gpu_mean,
            "gpu_median": gpu_median,
            "timestamp": time.time(),
            "time elapsed": time_elapsed
        }

        # Load existing data if file exists
        if os.path.exists(DATA_FILE):
            with open(DATA_FILE, "r") as f:
                try:
                    data = json.load(f)
                    if not isinstance(data, list):
                        data = [data]
                except json.JSONDecodeError:
                    data = []
        else:
            data = []

        data.append(new_entry)

        # Save back to file
        with open(DATA_FILE, "w") as f:
            json.dump(data, f, indent=2)

        print(json.dumps(new_entry, indent=2))

monitor = LoadMonitor()

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ["start", "stop"]:
        print("Usage: system_load.py [start|stop]")
        sys.exit(1)
    if sys.argv[1] == "start":
        monitor.start()
        while monitor.running:
            time.sleep(1)
    elif sys.argv[1] == "stop":
        monitor.stop()

if __name__ == "__main__":
    main()
