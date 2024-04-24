#!/usr/bin/python3

import subprocess
import argparse
import re
import hashlib
# this requires progressbar2 to be installed in the root environment. e.g.:
# sudo su
# python3 -m pip install progressbar2
from progressbar.bar import ProgressBar

def run_tool(args, check_str=None, debug=False):
    result = subprocess.run(['sudo', 'python3', './jtag_tool.py', '-r', ] + args, capture_output=True, text=True)

    if check_str is not None:
        passing = False
    else:
        passing = True
    if result.returncode == 0:
        output = result.stderr
        lines = output.split('\n')
        for line in lines:
            if check_str is not None:
                if check_str in line:
                     passing = True
            if debug:
                print(line)
    else:
        passing = False
        print(result.stderr)

    return passing

def burn(binfile):
    errlist = []
    last_test = 0
    with open(binfile, 'r') as f: # makes a terrible assumption that the last arg is the filename...
        for line in f:
            match = re.match("\#([0-9]*).*", line)
            if match:
                last_test = int(match.group(1))
    print(f"Processing tex file with {last_test} commands:")

    process = subprocess.Popen(['sudo', 'python3', './jtag_tool.py', '-f', binfile], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    progress = ProgressBar(min_value=0, max_value=last_test, prefix='Writing...').start()
    with open('burn.log', 'w') as f:
        while True:
            output_line = process.stderr.readline()
            if output_line == '' and process.poll() is not None:
                break
            if output_line:
                f.write(output_line)
            if '[ERROR]' in output_line:
                errlist += [output_line]
            
            match = re.match(".*Test:([0-9]*).*", output_line)
            if match:
                progress.update(int(match.group(1)))
                

    process.stdout.close()
    process.stderr.close()
    progress.finish()
    if len(errlist) > 0:
        for err in errlist:
            print(err)
        print(f"{len(errlist)} errors found")
    else:
        print("No errors found")

def pad_to_boundary(file_path, boundary=32):
    with open(file_path, 'rb') as file:
        content = file.read()

        # Calculate the padding needed
        padding_length = boundary - (len(content) % boundary)

        # If content length is already aligned, no padding needed
        if padding_length == boundary:
            return

        # Add padding
        padding = b'\x00' * padding_length

        # Write the padded content to a new file
        with open(file_path, 'wb') as padded_file:
            padded_file.write(content + padding)
   
def auto_int(x):
    return int(x, 0)

def main():
    parser = argparse.ArgumentParser(description="Program RRAM with a binary file")
    parser.add_argument(
        "-f", "--file", required=True, help="File to program", type=str
    )
    parser.add_argument(
        "-d", "--debug", help="turn on debugging spew", default=False, action="store_true"
    )
    parser.add_argument(
        "--offset", type=auto_int, help="Offset of file to write", default=0x20_0000
    )

    args = parser.parse_args()
    if args.debug:
       logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

    # check the hash of what we're programming - useful for debugging e.g. file sync errors
    print("File checksum:")
    try:
        with open(args.file, 'rb') as f:
            hash_md5 = hashlib.md5()
            hash_md5.update(f.read())
            print(hash_md5.hexdigest())
    except IOError as e:
        return f'Error opening or reading file: {e}'

    print("Ensuring voltages...")
    result = subprocess.run(['python3', './axp223.py'], capture_output=True, text=True)
    if result.returncode == 0:
        for line in result.stdout.split('\n'):
            print(line)
    else:
         print("Couldn't configure voltages")
         exit(1)

    print("Checking scan chain...")
    if not run_tool(['-f', './ipt_read_id.tex', '-r'], check_str='Test:0 Passed! Expected: 0x10102001 = result: 0x10102001'):
        print("Chip not in test mode, check SW4-1")
        exit(1)

    pad_to_boundary(args.file, boundary=32)

    print(f"Converting file to load at {args.offset:x}...")
    if not run_tool(['-f', args.file, '--offset', f'{args.offset}']):
        print("Couln't convert binary file")
        exit(1)

    print("Programming...")
    burn(re.sub(r'\.[^.]*$', '.tex', args.file))

    print("Conversion and programming finished!")

if __name__ == "__main__":
    main()
