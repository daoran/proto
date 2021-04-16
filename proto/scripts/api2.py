#!/usr/bin/env python3


def form_docstring(string_list):
    retval = []

    string_block = "".join(string_list)
    for line in string_block.split('\n'):
        line = line.strip()
        if len(line) == 0:
            continue
        elif line[0:3] == "/**":
            continue
        elif line[0:2] == "*/":
            continue
        elif line[0] == "*":
            retval.append(line[1:].strip())

    return "\n".join(retval)


def form_func_signature(func_sig):
    return "".join(func_sig).replace("{", "").rstrip() + ";"


def extract_section_name(s):
    s = s.replace("*", "")
    s = s.replace("/", "")
    s = s.strip()
    return s


def check_src_line(line):
    if len(line.strip()) and line.strip()[0] == "#":
        return False
    if line.strip() in ["/*", "/** @} */"]:
        return False

    return True


def parse_source(src_file):
    data = {}

    docstr_mode = False
    docstr = []

    keep_func = False
    func_sig = []

    for line in src_file:
        if check_src_line(line) is False:
            continue

        # Parse docstring
        if line.strip() == "/**":
            docstr_mode = True
        if docstr_mode:
            docstr.append(line)
        if line.strip() == "*/" and docstr_mode:
            docstr_mode = False

            ds = "".join(docstr).rstrip()
            if ds.find("@defgroup") > 0:
                docstr.clear()
            elif ds.find("@ingroup") > 0:
                docstr.clear()
            else:
                keep_func = True
                continue

        # Parse function signature
        ls = line.strip()
        if keep_func and len(ls) and ls[-1] not in ["{", "}", ",", ";"]:
            keep_func = False
            docstr.clear()
            func_sig.clear()

        elif keep_func and len(ls) and ls[-1] in ["{", "}", ",", ";"]:
            func_sig.append(line)
            if ls[-1] in [";", "{"]:
                # Keep track of function docstring and signature
                doc = {}
                doc["docstr"] = form_docstring(docstr)
                doc["func_sig"] = form_func_signature(func_sig)
                data[doc['func_sig']] = doc

                # Reset
                docstr.clear()
                func_sig.clear()
                keep_func = False
                docstr_mode = False


    return data


def check_header_line(line):
    ignore = ["#include"]

    if len(line.strip()) and line.strip().split()[0] in ignore:
        return False
    if line.strip() in ["/*", "/** @} */"]:
        return False

    return True


def parse_header(header_file):
    data = {}

    docstr_mode = False
    docstr = []

    section_mode = False
    section = []
    sections_all = []

    keep = False
    macro = []
    macros_all = []

    for line in header_file:
        if check_header_line(line) is False:
            continue

        # Extract sections
        ls = line.strip().replace("\\", "")
        if ls[0:5] == "/****":
            section_mode = True
            section.append(ls)

        elif section_mode:
            section.append(ls)

        if section_mode and ls[len(line) - 7:] == "****/":
            section_mode = False
            section_str = "\n".join(section)
            data = {}
            data['level'] = 1
            data['text'] = section_str
            data['name'] = extract_section_name(section_str)
            sections_all.append(data)
            section.clear()
            continue
        elif section_mode and ls[-2:] == '*/':
            section_mode = False
            section_str = "\n".join(section)
            data = {}
            data['level'] = 2
            data['text'] = section_str
            data['name'] = extract_section_name(section_str)
            sections_all.append(data)
            section.clear()
            continue

        # # Extract macros
        # if ls == "/**":
        #     docstr_mode = True
        #     docstr.append(line)
        # elif ls == "*/" and docstr_mode:
        #     docstr_mode = False
        #     docstr.append(line)
        #
        #     ds = "".join(docstr).rstrip()
        #     if ds.find("@defgroup") > 0:
        #         keep = False
        #         docstr.clear()
        #     elif ds.find("@ingroup") > 0:
        #         keep = False
        #         docstr.clear()
        #     else:
        #         keep = True
        #         continue
        #
        # elif keep and ls[0:7] == "#define":
        #     if len(macro) == 0:
        #         macro.append(ls.rstrip())
        #     continue
        #
        # elif keep:
        #     if len(ls) == 0 or (len(ls) and ls[-1] == ')'):
        #         data = {}
        #         data["docstr"] = "".join(docstr).rstrip()
        #         data["macro"] = "".join(macro)
        #         data["section_name"] = sections_all[-1]['name']
        #         macros_all.append(data)
        #
        #         docstr_mode = False
        #         keep = False
        #         docstr.clear()
        #         macro.clear()
        #     continue

        # elif keep is False and ls[0:7] == "#define":
        #     macro_split = ls.split("///<")
        #     if len(macro_split) > 1:
        #         macro = macro_split[0].strip()
        #         docstr = macro_split[1].strip()
        #         print(macro, docstr)

        print(line.rstrip())



    import pprint
    # pprint.pprint(macros_all)
    pprint.pprint(sections_all)

    return (sections_all, macros_all)



src_file = open('zero/zero.c', 'r')
header_file = open('zero/zero.h', 'r')
# data = parse_source(src_file)
(sections_all, macros_all) = parse_header(header_file)

section_names = [section['name'] for section in sections_all]





# import pprint
# pprint.pprint(data)

# for docstr in data:
#     print("-" * 80)
#     print("%s" % docstr["func_sig"])
#     print()
#     print("%s" % docstr["docstr"])
#     print("-" * 80)

# print('\n'.join(doc))
