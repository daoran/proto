import os
import html
import shutil
import textwrap
from glob import glob

import CppHeaderParser
import markdown
from jinja2 import Template

# GLOBAL VARIABLES
include_path = "../../include/prototype"
docs_path = "../../docs"
api_path = os.path.join(docs_path, "api")
header_impl_pattern = "_impl.hpp"
template_file = "template.html"
index_file = "index.html"
readme_file = "../../README.md"
md = markdown.Markdown(extensions=["fenced_code"])


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

    # Strip comments
    data = data["doxygen"]
    data = data.replace("/**", "")
    data = data.replace("*/", "")

    lines = data.split("\n")
    output = ""
    for line in lines:
        output += line[2:] if "*" in line[:2] else line
        output += "\n"

    # Search and replace doxygen keywords
    output = output.replace("@returns", "\n**Returns**")
    output = output.replace("@return", "\n**Returns**")
    output = output.replace("@param", "- ")
    return md.convert(output)


class CppObj:
    def __init__(self, class_name, data):
        # Class type and name
        cl = data.classes[class_name]
        self.cls_type = cl["declaration_method"]
        self.cls_name = cl["name"]
        self.cls_template = cl["template"] if "template" in cl else None
        self.cls_doc = process_docstring(cl)

        # Member variables
        self.properties = []
        for prop in cl["properties"]["public"]:
            self.properties.append({"type": prop["type"],
                                    "name": prop["name"]})

        # Member functions
        self.methods = []
        for method_data in cl["methods"]["public"]:
            self.methods.append(self._process_method(method_data))

    def _process_method(self, data):
        params = []

        for param in data["parameters"]:
            params.append({"type": param["type"], "name": param["name"]})

        return {
            "template": data["template"] if data["template"] else None,
            "destructor": data["destructor"],
            "constructor": data["constructor"],
            "returns": data["returns"],
            "type":  data["type"] if "type" in data else None,
            "name": data["name"],
            "params": params
        }

    def _build_method_str(self, func):
        output = ""
        output += func["template"] if func["template"] else ""

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
        output += ";"

        return textwrap.indent(clang_format(output), "  ")

    def __str__(self):
        output = ""
        output += self.cls_template + "\n" if self.cls_template else ""
        output += self.cls_type + " " + self.cls_name + " {\n"
        for var in self.properties:
            output += "  " + var["type"] + " " + var["name"] + ";\n"

        if len(self.methods):
            output += "\n"
        nb_methods = len(self.methods)
        for i in range(nb_methods):
            func = self.methods[i]
            output += self._build_method_str(func)
            if func["constructor"] is False and (i + 1) < nb_methods:
                output += "\n"
        output += "};"

        return html.escape(output)

    def data(self):
        return {"api": str(self), "doc": self.cls_doc}


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
            "template": data["template"] if data["template"] else None,
            "returns": data["returns"].replace(" : : ", "::"),
            "doxygen": data["doxygen"] if "doxygen" in data else None,
            "name": data["name"],
            "params": params
        }

    def _func_str(self, func):
        # Return type and function name
        fstr = ""
        fstr += func["template"] if func["template"] else ""
        fstr += func["returns"] + " " + func["name"]

        # Params
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

        return html.escape(fstr)

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
    t = Template(open(template_file).read())
    html = t.render(classes=classes, functions=functions.data())
    html_file = open(output_path, "w")
    html_file.write(html)
    html_file.close()


# def get_header_files()

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
    output_paths.append(os.path.join(api_path, basepath))

# Prepare output dirs
for path in output_paths:
    basedir = os.path.dirname(path)
    if os.path.exists(basedir) is False:
        os.makedirs(basedir)

# Render docs
nb_header_files = len(header_files)
for i in range(nb_header_files):
    render(header_files[i], output_paths[i])

# header = "../../include/prototype/driver/imu/mpu6050.hpp"
# dest = "../../docs/api/driver/imu/mpu6050.html"
# render(header, dest)

# Sidebar file
sidebar_path = os.path.join(docs_path, "sidebar.html")
sidebar_file = open(sidebar_path, "w")

sidebar_file.write("<h1><a href='.'>prototype</a></h1>\n\n")
sidebar_file.write("<h2>API:</h2>\n")
sidebar_file.write("<ul>\n")

nb_header_files = len(header_files)
current_module = ""
current_level = 0
for path in output_paths:
    relpath = path.replace(api_path, "")
    if relpath[0] == "/":
        relpath = relpath[1:]

    blocks = relpath.split("/")
    if len(blocks) == 1:
        continue

    if blocks[-1][-5:] != "html":
        fn = ".".join(blocks).replace(".html", "")
        sidebar_file.write("  <li><a href='#%s'>%s</a></li>\n" % (fn, fn))

sidebar_file.write("</ul>\n")
sidebar_file.close()


# Index file
shutil.copyfile(index_file, os.path.join(docs_path, index_file))


# Readme file
readme = open(readme_file, mode="r", encoding="utf-8").read()
readme_html_path = os.path.join(docs_path, "README.html")
readme_html_file = open(readme_html_path, "w")
readme_html_file.write(md.convert(readme))
readme_html_file.close()
