#!/usr/bin/env python3
import argparse

if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      prog='make_unittest.py',
      description='Makes a unittest from a single self-contained C header file',
  )

  parser.add_argument('-in', '--infile', required=True)
  parser.add_argument('-test', '--unittest', default=False)
  args = parser.parse_args()

  guard_prefix = args.infile.replace(".h", "").upper()
  outfile = args.infile.replace(".h", ".c")
  if args.unittest:
    outfile = "test_" + outfile

  unittest = open(outfile, "w")
  unittest.write(f"#define {guard_prefix}_IMPLEMENTATION\n")
  if args.unittest:
    unittest.write(f"#define {guard_prefix}_UNITTEST\n")
  unittest.write(f"#include \"{args.infile}\"\n")
  unittest.close()
