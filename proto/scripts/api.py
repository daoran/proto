#!/usr/bin/env python3
import os
from bs4 import BeautifulSoup
from jinja2 import Template

DOXYGEN_XML_PATH = "docs/xml"


def list_groups(path):
    files = []
    for f in os.listdir(path):
        file_name, file_ext = os.path.splitext(f)
        if file_name[0:5] == "group":
            files.append(os.path.join(path, f))
    return files


def form_func_signature_str(func_def, func_params):
    indent_size = len(func_def) + 1
    func_str = ""
    func_str += func_def
    func_str += "("

    for i in range(0, len(func_params)):
        param = func_params[i]
        if i != 0:
            func_str += ' ' * indent_size

        func_str += param

        if (i + 1) != len(func_params):
            func_str += ',\n'
        else:
            func_str += ')'

    return func_str


def parse_parameteritem(xml):
    direction = None
    para = None

    if xml.parameternamelist.parametername:
        if xml.parameternamelist.parametername.has_attr('direction'):
            direction = xml.parameternamelist.parametername['direction']

            if direction == 'in':
                direction = '[in]'
            elif direction == 'out':
                direction = '[out]'
            elif direction == 'inout':
                direction = '[in, out]'

    if xml.parameterdescription:
        para = parse_para(xml.parameterdescription.para)

    return {'direction': direction, 'para': para}


def parse_parameterlist(xml):
    param_list = []

    for item in xml.find_all('parameteritem'):
        retval = parse_parameteritem(item)
        if retval:
            param_list.append(param_list)

    return param_list


def parse_itemizedlist(xml):
    item_list = []

    for item in xml.find_all('listitem'):
        if item.para:
            retval = parse_para(item.para)
            if retval:
                item_list.append(retval)

    return item_list


def parse_formula(xml):
    return "".join(xml.strings).strip()


def parse_simplesect(xml):
    return {"returns": parse_para(xml)}


def parse_para(xml):
    retval = ""

    if xml.simplesect:
        retval = parse_simplesect(xml.simplesect)

    elif xml.itemizedlist:
        retval = parse_itemizedlist(xml.itemizedlist)

    if xml.parameterlist:
        retval = parse_parameterlist(xml.parameterlist)

    elif xml.formula:
        retval = parse_formula(xml.formula)

    else:
        retval = "".join(xml.strings).strip()
    return retval


def parse_detaileddescrption(xml):
    description = []

    for child in xml.find('detaileddescription'):
        print(child)
        if child.name == 'para':
            retval = parse_para(child)
            if len(retval):
                description.append(retval)

    return description


def parse_briefdescrption(xml):
    description = []

    for child in xml.find('briefdescription'):
        if child.name == 'para':
            retval = parse_para(child)
            if retval:
                description.append(retval)

    return description


def parse_memberdef(xml):
    func = {}
    func['def'] = ' '.join(memberdef.definition.strings)
    func['params'] = ' '.join(memberdef.argsstring.strings)
    func['params'] = func['params'].replace('(', '')
    func['params'] = func['params'].replace(')', '')
    func['params'] = func['params'].strip()
    func['params'] = func['params'].split(', ')
    func['signature'] = form_func_signature_str(func['def'], func['params'])
    func['brief'] = parse_briefdescrption(memberdef)
    func['description'] = parse_detaileddescrption(memberdef)

    import pprint
    pprint.pprint(func)


group_files = list_groups(DOXYGEN_XML_PATH)
xml_file = open(group_files[3], 'r')
soup = BeautifulSoup(xml_file, 'lxml')
elem = soup

for memberdef in soup.compounddef.sectiondef.find_all('memberdef'):
    parse_memberdef(memberdef)

    # text = parse_briefdescrption(memberdef)
    # if text:
    #     for t in text:
    #         print(t)
    #         print()



# template = Template('''\
# {% for func in funcs %}
# .. code-block::
#
#   {{ func['signature'] | replace('\n', '\n  ') }}
#
# {% if func['brief'] %}{{ func['brief'] }}{% endif %}
# {% if func['returns'] %}{{ func['returns'] }}{% endif %}
#
# {% endfor %}
# ''')
# api_doc = template.render(funcs=funcs)
# api = open('api.rst', 'w')
# api.write(api_doc)
# api.close()
