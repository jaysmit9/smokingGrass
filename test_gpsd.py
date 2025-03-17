#!/usr/bin/env python3

import socket
import time

def test_gpsd_connection():
    """Simple test to verify GPSD connection"""
    try:
        # Connect to GPSD
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 2947))
        print("Successfully connected to GPSD")
        
        # Send VERSION command
        print("Sending VERSION command...")
        sock.send(b'?VERSION;\n')
        
        # Wait for response
        time.sleep(0.5)
        response = sock.recv(4096).decode('utf-8').strip()
        print(f"Response: {response}")
        
        # Send DEVICES command
        print("\nSending DEVICES command...")
        sock.send(b'?DEVICES;\n')
        
        # Wait for response
        time.sleep(0.5)
        response = sock.recv(4096).decode('utf-8').strip()
        print(f"Response: {response}")
        
        # Send WATCH command
        print("\nSending WATCH command...")
        sock.send(b'?WATCH={"enable":true,"json":true};\n')
        
        # Wait for response
        time.sleep(0.5)
        response = sock.recv(4096).decode('utf-8').strip()
        print(f"Response: {response}")
        
        # Wait for a TPV report
        print("\nWaiting for TPV report (will wait 5 seconds max)...")
        sock.settimeout(5.0)
        try:
            response = sock.recv(4096).decode('utf-8').strip()
            print(f"Data received: {response}")
        except socket.timeout:
            print("No TPV report received in 5 seconds")
        
        sock.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_gpsd_connection()