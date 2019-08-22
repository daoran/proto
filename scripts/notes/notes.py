import os
import glob

import markdown
from jinja2 import Template

# GLOBAL VARIABLES
docs_path = "docs"
build_path = "docs/build"
template_file = "scripts/notes/template.html"
macros_file = "scripts/notes/macros.tex"
md = markdown.Markdown(extensions=["fenced_code"])


def get_notes(docs_path):
    note_files = []
    for x in os.walk(docs_path):
        for y in glob.glob(os.path.join(x[0], '*.md')):
            note_files.append(y)
    return sorted(note_files)


def escape_html(s):
    return s.replace("\\", "&#92;").replace("_", "&#95;")


def unescape_html(s):
    return s.replace("&amp;#92;", "\\") .replace("&amp;#95;", "_")


def render_note(docs_path, template_file, macros_file, note_file):
    # Macros
    macros = open(macros_file, mode="r", encoding="utf-8").read()
    macros = "$$" + macros + "$$"

    # Convert note file to html
    content = macros + "\n"
    content += open(note_file, mode="r", encoding="utf-8").read()
    content = md.convert(escape_html(content))
    content = unescape_html(content)

    # Render template
    template = Template(open(template_file).read())
    html = template.render(content=content)

    # Output rendered html
    docs_path += "/" if docs_path[-1] != "/" else ""
    fname = note_file.replace(docs_path, "").replace("md", "html")
    output_path = os.path.join(build_path, fname)
    output_file = open(output_path, "w")
    output_file.write(html)
    output_file.close()


note_files = get_notes(docs_path)
import pprint
pprint.pprint(note_files)

for note_file in note_files:
    print("processing [%s]" % note_file)
    render_note(docs_path, template_file, macros_file, note_file)
