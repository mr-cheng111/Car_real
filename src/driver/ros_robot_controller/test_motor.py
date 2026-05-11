import serial
import time
import sys
import threading

def read_thread(ser):
    t = threading.current_thread()
    while getattr(t, "do_run", True):
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                hex_data = " ".join([f"{b:02X}" for b in data])
                print(f"[RECV] <= {hex_data}")
        except:
            break
        time.sleep(0.01)

def test_motor_direct():
    port_name = "/dev/ttyS0"
    baudrate = 115200

    try:
        ser = serial.Serial(port_name, baudrate, timeout=1)
        print(f"Serial port {port_name} opened successfully.")
        
        reader = threading.Thread(target=read_thread, args=(ser,))
        reader.do_run = True
        reader.start()

        # Command 1: Motor 1 and 2 start turning
        turn_cmd = bytes([0xAA, 0x55, 0x03, 0x0C, 0x01, 0x02, 0x01, 0x00, 0x00, 0x80, 0xBF, 0x02, 0x00, 0x00, 0x00, 0x40, 0xFB])
        print(f"\n[SEND] Turn Command => {' '.join([f'{b:02X}' for b in turn_cmd])}")
        ser.write(turn_cmd)
        
        # Let it run for 3 seconds
        print("Running for 3 seconds...")
        for i in range(3):
            time.sleep(1)
        
        # Command 2: Motor 1 and 2 stop
        #stop_cmd = bytes([0xAA, 0x55, 0x03, 0x02, 0x03, 0x05, 0xAD])
        #print(f"\n[SEND] Stop Command => {' '.join([f'{b:02X}' for b in stop_cmd])}")
        #ser.write(stop_cmd)
        
        time.sleep(0.5)
        reader.do_run = False
        reader.join()
        
        print("Test complete.")
        ser.close()

    except Exception as e:
        print(f"Error testing motors: {e}")

if __name__ == '__main__':
    test_motor_direct()
