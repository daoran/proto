import os
import textwrap
from glob import glob

import CppHeaderParser
from jinja2 import Template

# GLOBAL VARIABLES
include_path = "../include/prototype"
output_path = "../docs/html"
header_impl_pattern = "_impl.hpp"


def get_classes(data):
    if len(data.classes) == 0:
        return None
    else:
        return list(data.classes.keys())


def clang_format(data):
    cmd = 'echo "%s" | clang-format' % (data)
    return os.popen(cmd).read()


def process_docstring(data):
    if "doxygen" not in data or data["doxygen"] is None:
        return None

    data = data["doxygen"]
    data = data.replace("/**", "")
    data = data.replace("*/", "")

    lines = data.split("\n")
    output = ""
    for line in lines:
        if line == "" or line == " ":
            continue
        output += line[2:] if line[:2] == "* " else line
        output += "\n"
    output = output.strip()

    result = ""
    code_on = False
    for c in output:
        if c == "`" and code_on is False:
            result += "<code>"
            code_on = True
        elif c == "`" and code_on:
            result += "</code>"
            code_on = False
        else:
            result += c
    result += "\n"

    return result


class CppObj:
    def __init__(self, class_name, data):
        # Class type and name
        self.cls_type = data.classes[class_name]["declaration_method"]
        self.cls_name = data.classes[class_name]["name"]
        self.cls_signature = self.cls_type + " " + self.cls_name
        self.cls_doc = process_docstring(data.classes[class_name])

        # Member variables
        self.properties = []
        for prop in data.classes[class_name]["properties"]["public"]:
            self.properties.append({"type": prop["type"],
                                    "name": prop["name"]})

        # Member functions
        self.methods = []
        for method_data in data.classes[class_name]["methods"]["public"]:
            self.methods.append(self._process_method(method_data))

    def _process_method(self, data):
        params = []
        for param in data["parameters"]:
            params.append({"type": param["type"], "name": param["name"]})
        return {
            "destructor": data["destructor"],
            "constructor": data["constructor"],
            "returns": data["returns"],
            "type":  data["type"] if "type" in data else None,
            "name": data["name"],
            "params": params
        }

    def _build_method_str(self, func):
        output = ""

        # Constructor or destructor
        if func["destructor"]:
            output += "~" + func["name"]
        elif func["constructor"]:
            output += func["name"]
        else:
            output += func["returns"] + " " + func["name"]

        # Params
        output += "("
        nb_params = len(func["params"])
        for i in range(nb_params):
            param = func["params"][i]
            output += param["type"]
            if param["type"][-1] != "&":
                output += " "
            output += param["name"]
            if (i + 1) != nb_params:
                output += ", "
        output += ")"
        output += ";\n"

        return textwrap.indent(clang_format(output), "  ")

    def __str__(self):
        output = ""
        output += self.cls_type + " " + self.cls_name + " {\n"
        for var in self.properties:
            output += "  " + var["type"] + " " + var["name"] + ";\n"

        output += "\n"
        for func in self.methods:
            output += self._build_method_str(func)
        output += "};"

        return output

    def data(self):
        return {"signature": self.cls_signature,
                "api": str(self),
                "doc": self.cls_doc}


class Functions:
    def __init__(self, data):
        self.funcs = [self._process_function(f) for f in data.functions]

    def _process_function(self, data):
        params = []
        for param in data["parameters"]:
            param_type = param["type"]
            param_type = param_type.replace(" : : ", "::")
            params.append({"type": param_type, "name": param["name"]})

        return {
            "returns": data["returns"].replace(" : : ", "::"),
            "doxygen": data["doxygen"] if "doxygen" in data else None,
            "name": data["name"],
            "params": params
        }

    def _func_str(self, func):
        fstr = ""
        fstr += func["returns"] + " " + func["name"]

        fstr += "("
        nb_params = len(func["params"])
        for i in range(nb_params):
            param = func["params"][i]
            fstr += param["type"]
            if param["type"][-1] not in ["&", ":"]:
                fstr += " "

            fstr += param["name"]
            if (i + 1) != nb_params:
                fstr += ", "
        fstr += ")"
        fstr += ";\n"
        fstr = clang_format(fstr)

        return fstr

    def __str__(self):
        output = ""

        for func in self.funcs:
            output += self._func_str(func)
            doc = process_docstring(func)
            if doc is not None:
                output += doc

        return output

    def data(self):
        output = []
        for func in self.funcs:
            output.append({"api": self._func_str(func),
                           "doc": process_docstring(func)})
        return output


def render(header_path, output_path):
    print("Processing header [%s]" % header_path)
    header = CppHeaderParser.CppHeader(header_path)

    # Classes
    class_names = get_classes(header)
    classes = []
    if class_names is not None:
        for class_name in class_names:
            cpp_cls = CppObj(class_name, header)
            classes.append(cpp_cls.data())

    # Functions
    functions = Functions(header)

    # Render and save document
    t = Template(open("template.html").read())
    html = t.render(classes=classes, functions=functions.data())
    html_file = open(output_path, "w")
    html_file.write(html)
    html_file.close()


# import pprint
# pprint.pprint(header.classes["aprilgrid_t"])
# exit(0);

# Get all headers
if include_path[-1] != "/":
    include_path += "/"

header_files = []
for x in os.walk(include_path):
    for y in glob(os.path.join(x[0], '*.hpp')):
        header_files.append(y)
header_files = sorted(header_files)

# Remove header implementation files
for header in header_files:
    if header_impl_pattern in header:
        header_files.remove(header)

# Prepare output paths
output_paths = []
for header in header_files:
    basepath = header.replace(include_path, "").replace(".hpp", ".html")
    output_paths.append(os.path.join(output_path, basepath))

# Prepare output dirs
for path in output_paths:
    basedir = os.path.dirname(path)
    if os.path.exists(basedir) is False:
        os.makedirs(basedir)

# Render docs
nb_header_files = len(header_files)
for i in range(nb_header_files):
    render(header_files[i], output_paths[i])

# Sidebar file
sidebar_path = os.path.join(output_path, "sidebar.html")
sidebar_file = open(sidebar_path, "w")

sidebar_file.write("<h1>prototype</h1>\n\n")
sidebar_file.write("<h2>API:</h2>\n")
sidebar_file.write("<ul>\n")

nb_header_files = len(header_files)
for path in output_paths:
    relpath = path.replace(output_path, "")
    if relpath[0] == "/":
        relpath = relpath[1:]

    blocks = relpath.split("/")
    if len(blocks) == 1:
        continue

    filename = ".".join(blocks).replace(".html", "")
    sidebar_file.write("  <li><a href='#%s'>%s</a></li>\n" % (filename, filename))

sidebar_file.write("</ul>\n")
sidebar_file.close()
