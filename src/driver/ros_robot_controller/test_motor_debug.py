import serial
import time
import sys
import threading
import datetime

def log(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}")

def read_thread(ser):
    t = threading.current_thread()
    log("[THREAD] Read thread started.")
    bytes_received = 0
    while getattr(t, "do_run", True):
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                bytes_received += len(data)
                hex_data = " ".join([f"{b:02X}" for b in data])
                log(f"[RECV] <= {hex_data} (Total read: {bytes_received} bytes)")
        except Exception as e:
            log(f"[THREAD] Exception in read_thread: {e}")
            break
        time.sleep(0.01)
    log("[THREAD] Read thread exiting.")

def test_motor_direct():
    port_name = "/dev/ttyS0"
    baudrate = 115200

    log(f"Configuring serial connection...")
    log(f" - Port: {port_name}")
    log(f" - Baudrate: {baudrate}")

    try:
        ser = serial.Serial(port_name, baudrate, timeout=1)
        log(f"Serial port {port_name} opened successfully.")
        
        # Flush input/output buffers before starting
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        log("Serial buffers flushed.")
        
        reader = threading.Thread(target=read_thread, args=(ser,))
        reader.do_run = True
        reader.start()

        # Command 1: Motor 1 and 2 start turning
        turn_cmd = bytes([0xAA, 0x55, 0x03, 0x0C, 0x01, 0x02, 0x01, 0x00, 0x00, 0x80, 0xBF, 0x02, 0x00, 0x00, 0x00, 0x40, 0xFB])
        log(f"[SEND] Turn Command => {' '.join([f'{b:02X}' for b in turn_cmd])} (Len: {len(turn_cmd)} bytes)")
        sent_len = ser.write(turn_cmd)
        ser.flush()
        log(f"[SEND] Successfully wrote {sent_len} bytes to tx buffer.")
        
        # Let it run for 3 seconds
        log("Running for 3 seconds. Waiting for motor response...")
        for i in range(30):
            time.sleep(0.1)
            # Just blocking here, read thread works in background
        
        # Command 2: Motor 1 and 2 stop
        stop_cmd = bytes([0xAA, 0x55, 0x03, 0x02, 0x03, 0x05, 0xAD])
        log(f"[SEND] Stop Command => {' '.join([f'{b:02X}' for b in stop_cmd])} (Len: {len(stop_cmd)} bytes)")
        sent_len = ser.write(stop_cmd)
        ser.flush()
        log(f"[SEND] Successfully wrote {sent_len} bytes to tx buffer.")
        
        log("Waiting 0.5s for final responses...")
        time.sleep(0.5)
        reader.do_run = False
        reader.join()
        
        log("Test complete. Closing port.")
        ser.close()

    except Exception as e:
        log(f"[ERROR] Exception testing motors: {e}")

if __name__ == '__main__':
    #test_motor_direct()
    ser = serial.Serial("/dev/ttyS0", 115200, timeout=1)
    data = bytes.fromhex("AA 55 03 0C 01 02 01 00 00 80 BF 02 00 00 00 40 FB")
    ser.write(data)