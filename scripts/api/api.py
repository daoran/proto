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
            print(prop)
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
            if var["name"]:
                output += "  " + var["type"] + " " + var["name"] + ";\n"
            else:
                output += "  " + var["type"] + ";\n"

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


def render_api(header_path, output_path):
    header = CppHeaderParser.CppHeader(header_path)
    print("Processing header [%s]" % header_path)

    # Documentation
    doc = parse_docstring(header_path, "<doc>")

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
    inc_start = header_path.find("include")
    inc_end = inc_start + len("include") + 1
    header_path = header_path[inc_end:]

    t = Template(open(template_file).read())
    html = t.render(header_path=header_path,
                    doc=doc,
                    classes=classes,
                    functions=functions.data())
    html_file = open(output_path, "w")
    html_file.write(html)
    html_file.close()


def render_headers(header_files, output_paths):
    nb_header_files = len(header_files)
    for i in range(nb_header_files):
        render_api(header_files[i], output_paths[i])


def parse_docstring(header_file, tag):
    is_docstr = False
    docstr = []
    start_tag = "/*" + tag
    end_tag = tag + "*/"

    header = open(header_file, "r").read()
    for line in header.split("\n"):
        if line.strip() == start_tag:
            is_docstr = True
            continue
        elif line.strip() == end_tag:
            break
        elif is_docstr:
            docstr.append(line)
    docstr = "\n".join(docstr)

    if len(docstr) != 0 and docstr != " ":
        return md.convert(docstr)
    else:
        return None


def render_sidebar(sidebar_header, docs_path, header_files):
    # Get sidebar content
    docstr = parse_docstring(sidebar_header, "<sidebar_doc>")
    if docstr is None:
        raise RuntimeError("No docstring in [%s]?" % sidebar_header)

    # Sidebar file
    sidebar_path = os.path.join(docs_path, "sidebar.html")
    sidebar_file = open(sidebar_path, "w")
    sidebar_file.write(docstr)
    sidebar_file.close()


def render_readme(readme_file, docs_path):
    readme = open(readme_file, mode="r", encoding="utf-8").read()
    readme_html_path = os.path.join(docs_path, "README.html")
    readme_html_file = open(readme_html_path, "w")
    readme_html_file.write(md.convert(readme))
    readme_html_file.close()


def render_index(index_file, docs_path):
    # Index file
    shutil.copyfile(index_file, os.path.join(docs_path, index_file))


def get_header_files(include_path):
    if include_path[-1] != "/":
        include_path += "/"

    # Get all headers
    header_files = []
    for x in os.walk(include_path):
        for y in glob(os.path.join(x[0], '*.hpp')):
            header_files.append(y)
    header_files = sorted(header_files)

    # Remove header implementation files
    for header in header_files:
        if header_impl_pattern in header:
            header_files.remove(header)

    return header_files


def prepare_destination(header_files, include_path, api_path):
    # Prepare output paths
    output_paths = []
    if api_path[-1] != "/":
        api_path += "/"

    for header in header_files:
        basepath = header.replace(include_path, "").replace(".hpp", ".html")
        output_paths.append(api_path + basepath)

    # Prepare output dirs
    for path in output_paths:
        basedir = os.path.dirname(path)
        if os.path.exists(basedir) is False:
            os.makedirs(basedir)

    return output_paths


# MAIN
header_files = get_header_files(include_path)
output_paths = prepare_destination(header_files, include_path, api_path)
render_headers(header_files, output_paths)
render_sidebar(include_path + "/prototype.hpp", docs_path, header_files)
render_readme(readme_file, docs_path)
render_index(index_file, docs_path)

# header = "../../include/prototype/calib/aprilgrid.hpp"
# dest = "../../docs/api/calib/aprilgrid.html"
# render_api(header, dest)
