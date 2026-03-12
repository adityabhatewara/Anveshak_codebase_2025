import tkinter as tk
import subprocess
import threading
import time

# --- CONFIGURATION ---
# UPDATE THESE IPS TO MATCH YOUR EXACT SETUP
DEVICES = [  
    {"name": "Base Mikrotik","ip": "10.42.0.99"},
    {"name": "Rover Mikrotik","ip": "10.42.0.100"},
    {"name": "Jetson Orin",   "ip": "10.42.0.253"},
    {"name": "Imou Camera",   "ip": "10.42.0.69"},
    {"name": "Adeesh's Laptop","ip": "10.42.0.7"},
]

class NetworkMap:
    def __init__(self, root):
        self.root = root
        self.root.title("Rover Link Status")
        self.root.geometry("350x300")
        self.root.configure(bg="#111")
        
        self.indicators = {}

        # HEADER
        tk.Label(root, text="NETWORK STATUS", font=("Consolas", 14, "bold"), 
                 fg="#0f0", bg="#111", pady=10).pack()

        # DEVICE LIST
        for dev in DEVICES:
            frame = tk.Frame(root, bg="#222", pady=2)
            frame.pack(fill="x", padx=10, pady=3)

            # Device Name
            tk.Label(frame, text=dev["name"], font=("Arial", 11), 
                     fg="white", bg="#222", width=15, anchor="w").pack(side="left", padx=5)
            
            # IP Address
            tk.Label(frame, text=dev["ip"], font=("Consolas", 9), 
                     fg="#888", bg="#222", width=12, anchor="w").pack(side="left")

            # Status Light
            canvas = tk.Canvas(frame, width=20, height=20, bg="#222", highlightthickness=0)
            light = canvas.create_oval(3, 3, 17, 17, fill="grey")
            canvas.pack(side="right", padx=10)
            
            self.indicators[dev["ip"]] = (canvas, light)

        # Start Monitor Thread
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()

    def ping(self, ip):
        try:
            # Linux Ping Command (-c 1 = count 1, -W 1 = wait 1 sec)
            return subprocess.call(['ping', '-c', '1', '-W', '1', ip], 
                                   stdout=subprocess.DEVNULL, 
                                   stderr=subprocess.DEVNULL) == 0
        except: return False

    def loop(self):
        while self.running:
            for dev in DEVICES:
                ip = dev["ip"]
                is_up = self.ping(ip)
                
                # Update GUI safely
                color = "#00FF00" if is_up else "#FF0000" # Green vs Red
                canvas, light = self.indicators[ip]
                try: canvas.itemconfig(light, fill=color)
                except: pass
            time.sleep(1)

if __name__ == "__main__":
    root = tk.Tk()
    app = NetworkMap(root)
    root.mainloop()