#!/usr/bin/env python3
"""
Pymavlink Directory Uploader Script

This script uses the MAVFTP class from the pymavlink library to upload the
contents of a local directory to the /APM/scripts/ directory on a vehicle
running ArduPilot.

This version is specifically adapted to use the MAVFTP class API provided
by the user.

It automates the process of:
1. Connecting to the vehicle.
2. Attempting to create '/APM' and '/APM/scripts' and handling 'FileExists'
   errors gracefully.
3. Uploading each file from the specified local source directory individually.
4. Terminating the connection gracefully.

Dependencies:
- Pymavlink (with mavftp module)

Example Usage:
1. Make the script executable (on Linux/macOS):
   chmod +x update_scripts.py

2. Sync a local 'scripts' directory to a SITL instance:
   ./update_scripts.py --connect udp:127.0.0.1:14550 ./my_lua_scripts

3. Sync a directory to a vehicle connected via a serial port:
   python update_scripts.py --connect /dev/ttyACM0 /path/to/local/scripts
"""

import argparse
import os
import sys
import time
import traceback  # Import the traceback module
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, FtpError, DirectoryEntry

def sync_directory(connect_str: str, source_dir: str):
    """
    Connects to a vehicle and syncs a local directory to the vehicle's
    /APM/scripts/ directory using the MAVFTP class.
    """
    if not os.path.isdir(source_dir):
        print(f"Error: Source directory not found at '{source_dir}'")
        sys.exit(1)

    files_to_upload = sorted([
        (f, os.path.join(source_dir, f))
        for f in os.listdir(source_dir)
        if os.path.isfile(os.path.join(source_dir, f))
    ])

    if not files_to_upload:
        print("\nSource directory is empty. No files to upload.")
        return

    master = None
    try:
        print(f"Connecting to vehicle on: {connect_str}")
        os.environ['MAVLINK20'] = '1'
        master = mavutil.mavlink_connection(connect_str, source_system=255)
        
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        print("Heartbeat received. Vehicle is connected.")

        # Add a delay to allow the vehicle to initialize its FTP server
        print("Waiting for vehicle to settle...")
        time.sleep(1)

        ftp = MAVFTP(master, master.target_system, master.target_component)

    except Exception as e:
        print(f"Failed to connect or initialize FTP: {e}")
        print("Please check the connection string and ensure the vehicle is powered and ready.")
        sys.exit(1)

    try:
        # --- Step 1: Ensure remote directories exist ---
        print("\n--- Ensuring remote directories exist ---")
        
        if not directory_exists(ftp, "/APM"):
            print("Directory '/APM' not found, creating it...")
            result = ftp.cmd_mkdir(["/APM"])
            if result.error_code != FtpError.Success:
                result.display_message()
                raise RuntimeError("Failed to create /APM directory")
        else:
            print("Directory '/APM' already exists.")

        if not directory_exists(ftp, "/APM/scripts"):
            print("Directory '/APM/scripts' not found, creating it...")
            result = ftp.cmd_mkdir(["/APM/scripts"])
            if result.error_code != FtpError.Success:
                result.display_message()
                raise RuntimeError("Failed to create /APM/scripts directory")
        else:
            print("Directory '/APM/scripts' already exists.")


        # --- Step 2: Upload each file individually ---
        print(f"\n--- Uploading {len(files_to_upload)} files ---")
        successful_uploads = 0
        for filename, local_path in files_to_upload:
            remote_path = f"/APM/scripts/{filename}"
            print(f"Uploading '{local_path}' to '{remote_path}'...")
            try:
                # Initiate the put command (this part is non-blocking and was already correct)
                ftp.cmd_put([local_path, remote_path])
                # Process replies until the operation is complete.
                result = ftp.process_ftp_reply('put', timeout=600)

                if result.error_code == FtpError.Success:
                    print("  [SUCCESS] Upload complete.")
                    successful_uploads += 1
                else:
                    print(f"\n  [FAILURE] Could not upload file '{filename}'.")
                    result.display_message()

            except Exception as e:
                # MODIFIED: Print the full stack trace for detailed debugging
                print("\n--- DETAILED EXCEPTION TRACE ---")
                traceback.print_exc()
                print("------------------------------------")
                print(f"\n  [FAILURE] An exception occurred during upload of '{filename}'. Reason: {e}")
                # Attempt to cancel any stuck operation
                ftp.cmd_cancel()
                ftp.process_ftp_reply('cancel', timeout=5)


        # --- Step 3: Final Report ---
        print("\n--- Sync Complete ---")
        print(f"Successfully uploaded {successful_uploads} of {len(files_to_upload)} files.")
        if successful_uploads == len(files_to_upload):
            print("[SUCCESS] All files were synced successfully.")
        else:
            print("[FAILURE] Some files failed to upload. Please review the log.")
            sys.exit(1)

    finally:
        if master and getattr(master, 'close', None):
            master.close()
            print("\nConnection closed.")

def directory_exists(ftp: MAVFTP, path: str) -> bool:
    """Checks if a directory exists by listing its parent."""
    parent_dir, target_name = os.path.split(path)
    if not parent_dir:
        parent_dir = "/"
    
    print(f"\n[DEBUG] Checking if directory '{target_name}' exists in '{parent_dir}'...")

    try:
        # CORRECTED: cmd_list is now a blocking call. Use its direct return value.
        result = ftp.cmd_list([parent_dir])

        if result.error_code != FtpError.Success:
            print(f"[DEBUG] Failed to list '{parent_dir}'. Assuming directory does not exist.")
            result.display_message()
            return False

        print(f"[DEBUG] Directory listing for '{parent_dir}':")
        if not ftp.list_result:
            print("[DEBUG]   (empty listing)")
        
        # The result is stored in the list_result attribute of the ftp object
        for entry in ftp.list_result:
            # ArduPilot FTP server entries might have null terminators or whitespace
            entry_name = entry.name.strip('\x00').strip()
            print(f"[DEBUG]   - Found entry: name='{entry.name}', stripped='{entry_name}', is_dir={entry.is_dir}")
            if entry_name == target_name and entry.is_dir:
                print(f"[DEBUG]   >>> Match found for '{target_name}'.")
                return True
        
        print(f"[DEBUG] No match found for '{target_name}'.")
        return False
    except Exception as e:
        print(f"[DEBUG] An exception occurred in directory_exists: {e}")
        return False

def main():
    """Parses command-line arguments and initiates the directory sync."""
    parser = argparse.ArgumentParser(
        description="Upload files from a local directory to /APM/scripts/ on an ArduPilot vehicle using Pymavlink.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--connect", required=True, help="Pymavlink connection string (e.g., udp:127.0.0.1:14550, /dev/ttyUSB0).")
    parser.add_argument("source_directory", help="Local directory of files to upload.")
    args = parser.parse_args()
    sync_directory(args.connect, args.source_directory)

if __name__ == "__main__":
    main()

