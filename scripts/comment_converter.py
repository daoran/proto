#/usr/bin/evn python3
src_file = open("proto/ubx.h", "r")
lines = src_file.readlines()

new_file = open("proto/ubx.h.new", "w")
for line in lines:
  if "/*" in line and "*/" in line:
    line = line.replace("/*", "//")
    line = line.replace("*/", "  ")
  new_file.write(line)
new_file.close()
