#/usr/bin/evn python3
src_file = open("proto/proto.c", "r")
lines = src_file.readlines()

new_file = open("proto/proto.c.new", "w")
for line in lines:
  if "/*" in line and "*/" in line:
    line = line.replace("/*", "//").replace("*/", "")
  new_file.write(line)
new_file.close()
