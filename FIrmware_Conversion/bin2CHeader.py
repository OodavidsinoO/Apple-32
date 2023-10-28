# Binary file to C header file converter
# Author: David Sin
# Date: 2023-10-28

import argparse

def bin2header(data, varName, lineLen):
    """ Convert binary data to C header file """
    hdr = "const unsigned char " + varName + "[] = {\n"
    for i in range(len(data)):
        if i % lineLen == 0:
            hdr += "    "
        hdr += "0x%02x" % data[i]
        if i < len(data) - 1:
            hdr += ","
        if i % lineLen == lineLen - 1:
            hdr += "\n"
        else:
            hdr += " "
    hdr += "};\n"
    hdr += "const unsigned int " + varName + "_len = " + str(len(data)) + ";\n"
    return hdr

def main():
    parser = argparse.ArgumentParser(description='Convert binary file to C header file')
    parser.add_argument('-i', '--input', help='input file', required=True)
    parser.add_argument('-o', '--output', help='output file', required=True)
    parser.add_argument('-n', '--name', help='variable name', required=True)
    parser.add_argument('-l', '--length', help='line length', required=False, default=16)
    args = parser.parse_args()

    with open(args.input, 'rb') as f:
        data = f.read()

    with open(args.output, 'w') as f:
        f.write(bin2header(data, args.name, args.length))

if __name__ == "__main__":
    main()