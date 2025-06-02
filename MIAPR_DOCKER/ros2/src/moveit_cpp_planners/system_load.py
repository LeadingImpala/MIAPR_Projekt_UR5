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
        # Sample every 0.5 seconds
        while self.running:
            cpu = psutil.cpu_percent(interval=None)
            gpus = GPUtil.getGPUs()
            gpu_load = 0.0
            if gpus:
                gpu_load = sum(gpu.load for gpu in gpus) / len(gpus) * 100  # convert to %
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

    def stop(self):
        if not self.running:
            print("Not running.")
            return
        self.running = False
        self.thread.join()
        # Calculate stats
        cpu_mean = sum(self.cpu_loads) / len(self.cpu_loads) if self.cpu_loads else 0
        gpu_mean = sum(self.gpu_loads) / len(self.gpu_loads) if self.gpu_loads else 0
        cpu_median = sorted(self.cpu_loads)[len(self.cpu_loads)//2] if self.cpu_loads else 0
        gpu_median = sorted(self.gpu_loads)[len(self.gpu_loads)//2] if self.gpu_loads else 0
        stats = {
            "cpu_mean": cpu_mean,
            "cpu_median": cpu_median,
            "gpu_mean": gpu_mean,
            "gpu_median": gpu_median,
        }
        # Save stats to file or print them
        with open(DATA_FILE, "w") as f:
            json.dump(stats, f)
        print(json.dumps(stats))

monitor = LoadMonitor()

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ["start", "stop"]:
        print("Usage: system_load.py [start|stop]")
        sys.exit(1)
    if sys.argv[1] == "start":
        monitor.start()
        # Keep the process alive until stopped
        while monitor.running:
            time.sleep(1)
    elif sys.argv[1] == "stop":
        monitor.stop()

if __name__ == "__main__":
    main()
