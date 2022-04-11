#/usr/bin/evn python3
src_file = open("proto/proto.h", "r")
lines = src_file.readlines()
src_file.close()

new_file = open("proto/proto.h", "w")
for line in lines:
  if "/**" not in line and "/*" in line and "*/" in line:
    line = line.replace("/*", "//")
    line = line.replace("*/", "  ")
  new_file.write(line)
new_file.close()
