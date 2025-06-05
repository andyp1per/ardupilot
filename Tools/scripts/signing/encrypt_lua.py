#!/usr/bin/env python3
'''
encrypt a lua script using Monocypher
'''

import sys
import os
import base64
import secrets

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='Lua encryption utility')
parser.add_argument("--uuid", type=str, default=None, help="UUID in hex format to encrypt the script with")
parser.add_argument("script", type=str, default=None, help="Script to encrypt")
parser.add_argument("key", type=str, default=None, help="Encryption key")
args = parser.parse_args()

try:
    import monocypher
except ImportError:
    print("Please install monocypher with: python3 -m pip install pymonocypher==3.1.3.2")
    sys.exit(1)

if monocypher.__version__ != "3.1.3.2":
    Logs.error("must use monocypher 3.1.3.2, please run: python3 -m pip install pymonocypher==3.1.3.2")
    sys.exit(1)

fname = args.script
bname = args.key
public_fname = "%s_public_key.dat" % bname

if args.uuid:
    nonce = bytes.fromhex(args.uuid)
    nonce = nonce + secrets.token_bytes(24-len(nonce))
else:
    nonce = secrets.token_bytes(24)

def decode_key(key):
    base64_data = key.split(":", 1)[1]
    return base64.b64decode(base64_data)
    
with open(public_fname, "r") as file:
    line = file.readline().strip()

public_key = decode_key(line)
encrypted_script_filename = os.path.splitext(fname)[0] + ".lxa"

script_file = open(fname, "rb")
msg = script_file.read()
mac, c = monocypher.lock(public_key, nonce, msg)

encrypted_script_file = open(encrypted_script_filename, "wb")
encrypted_script_file.write("LUA1.0".encode("utf-8"))
encrypted_script_file.write(mac)
encrypted_script_file.write(nonce)
encrypted_script_file.write(c)
encrypted_script_file.close()
