# /usr/bin/evn python3
from intelhex import IntelHex
import sys

# py3 .\bin2hex.py '"a.bin"' 40000 '"a_40000.hex"' 
def main(arg):
    ih=IntelHex()
    ih.loadbin(arg[1],offset=int(arg[2],16))
    ih.write_hex_file(arg[3])


if __name__ == '__main__':
	main(sys.argv) 
