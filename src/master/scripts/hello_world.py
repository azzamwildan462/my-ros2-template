#!/usr/bin/python3 


# -*- coding: utf-8 -*-

# Doing infinity loop with period 1 s for counting up 
# and printing the number of seconds passed since the start of the script 
import time
import sys
import os

def main():
    print("Hello World!")
    print("This is a test script to check if the package is working properly.")
    print("If you see this message, the package is working properly.")
    print("This script will count up from 0 to 10 and then exit.")
    print("Press Ctrl+C to exit the script at any time.")
    
    i = 0
    while True:
        print(f"Count: {i}")
        i += 2

        time.sleep(1)
    
    print("Exiting the script.")
    sys.exit(0)

if __name__ == "__main__":
    main()