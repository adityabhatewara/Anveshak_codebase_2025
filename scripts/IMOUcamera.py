from onvif import ONVIFCamera
import cv2
import threading

# --- CONFIGURATION ---
IP = "192.168.1.108"
PORT = 80             # ONVIF uses Port 80
USER = "admin"
PASS = "L26AF42F" # <--- REPLACE THIS

# RTSP Stream (Use subtype=1 for Low Latency)
RTSP_URL = f"rtsp://{USER}:{PASS}@{IP}:554/cam/realmonitor?channel=1&subtype=1"  ## put subtype = 0 for better queality (more latency)

# Global Variables
ptz = None
active_move = False

def setup_ptz():
    global ptz
    try:
        print(f"Connecting to ONVIF service at {IP}:{PORT}...")
        mycam = ONVIFCamera(IP, PORT, USER, PASS)
        
        # Create Media and PTZ services
        media = mycam.create_media_service()
        ptz = mycam.create_ptz_service()
        
        # Get the 'Profile Token' (The ID of the video stream to control)
        media_profile = media.GetProfiles()[0]
        token = media_profile.token
        
        print(f"✅ ONVIF Connected! Profile Token: {token}")
        return ptz, token
    except Exception as e:
        print(f"❌ ONVIF Connection Failed: {e}")
        return None, None

def move(ptz, token, direction):
    global active_move
    if active_move: return # Don't spam commands
    
    request = ptz.create_type('ContinuousMove')
    request.ProfileToken = token
    
    # X = Pan (Left/Right), Y = Tilt (Up/Down)
    # Speed range is usually -1.0 to 1.0
    if direction == 'Up':
        request.Velocity = {'PanTilt': {'x': 0, 'y': 1.0}}
    elif direction == 'Down':
        request.Velocity = {'PanTilt': {'x': 0, 'y': -1.0}}
    elif direction == 'Left':
        request.Velocity = {'PanTilt': {'x': -1.0, 'y': 0}}
    elif direction == 'Right':
        request.Velocity = {'PanTilt': {'x': 1.0, 'y': 0}}
        
    try:
        ptz.ContinuousMove(request)
        active_move = True
        print(f"Moving {direction}")
    except Exception as e:
        print(f"Move Error: {e}")

def stop(ptz, token):
    global active_move
    try:
        ptz.Stop({'ProfileToken': token})
        active_move = False
        print("Stopped")
    except:
        pass

def main():
    print("--- ROVER ONVIF CONTROLLER ---")
    
    # 1. Setup Control
    ptz_service, token = setup_ptz()
    if not ptz_service:
        print("Could not connect controls. Exiting.")
        return

    # 2. Setup Video
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("Controls: W/A/S/D to Move | SPACE to Stop | Q to Quit")
    print("Click the video window to drive!")

    current_key = None

    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        cv2.imshow("Rover Feed", frame)
        key = cv2.waitKey(1) & 0xFF

        # Logic: Move on key press, stop on Spacebar (or key release if implemented differently)
        if key == ord('w'): move(ptz_service, token, 'Up')
        elif key == ord('s'): move(ptz_service, token, 'Down')
        elif key == ord('a'): move(ptz_service, token, 'Left')
        elif key == ord('d'): move(ptz_service, token, 'Right')
        elif key == 32:       stop(ptz_service, token) # SPACEBAR TO STOP
        elif key == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()



