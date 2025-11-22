# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

# -- Project information -----------------------------------------------------

project = u'xyz'
copyright = u'2020, Chris Choi'
author = u'Chris Choi'

# The short X.Y version
version = u''
# The full version, including alpha/beta/rc tags
release = u'alpha'

# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.mathjax',
    'sphinx.ext.autosectionlabel',
]

mathjax3_config = {
    'tex': {
        'macros': {
            # General
            'Vec': [r'\mathbf{#1}', 1],
            'Mat': [r'\mathbf{#1}', 1],
            'real': r'\rm I\!R',
            'Real': [r'{\real^{#1}}', 1],
            'ones': r'{\Vec{1}}',
            'Ones': [r'{\Vec{1}_{#1\times#2}}', 2],
            'zeros': '{\Vec{0}}',
            'Zeros': [r'{\Vec{0}_{#1\times#2}}', 2],
            'Norm': [r'{\left\lVert{#1}\right\rVert}', 1],
            'Det': [r'{\text{det}(#1)}', 1],
            'I': r'{\Mat{I}}',
            # 'Skew': [r'{\lfloor #1 \enspace \times \rfloor}', 1],
            'Skew': [r'{\left(#1\right)^{\times}}', 1],
            'Min': [r'\text{min}(#1, #2)', 2],
            'Max': [r'\text{max}(#1, #2)', 2],
            'argmin': r'\mathop{\mathrm{argmin}',
            'Argmin': [r'\underset{#1}{\text{argmin }}', 1],
            'transpose': 'T',
            'Transpose': [r'{#1^{\transpose}}', 1],
            'Inv': [r'{#1^{-1}}', 1],
            'Trace': [r'\text{tr}(#1)', 1],
            'Rank': [r'\text{rank}(#1)', 1],
            'E': [r'\mathbb{E}\left[#1\right]', 1],
            'Bigslant': [r'{\left#1/#2\right}', 2],
            'cost': 'J',
            'error': r'{\Vec{e}}',
            'SO': [r'\text{SO}(#1)', 1],
            'so': [r'\mathfrak{so}(#1)', 1],
            'jac': r'\text{J}',
            # Probability
            'RV': [r'\mathbf{#1}', 1, r''],
            'covar': r'\mathbf{\Sigma}',
            # Transforms
            'frame': r'{\mathcal{F}}',
            'rot': r'{\Mat{C}}',
            'trans': r'{\Vec{r}}',
            'quat': r'{{\Vec{q}}}',
            'tf': r'{\Mat{T}}',
            'pt': r'{\Vec{r}}',
            'Rot': [r'{\rot_{#1#2}}', 2],
            'Trans': [r'{\trans_{#1#2}}', 2],
            'Quat': [r'{\quat_{#1#2}}', 2],
            'Tf': [r'{\tf_{#1#2}}', 2],
            'Pt': [r'{\pt_{#1#2}}', 2],
            # Standard terms
            'state': r'{\Vec{x}}',
            'pos': r'{\Vec{r}}',
            'vel': r'{\Vec{v}}',
            'acc': r'{\Vec{a}}',
            'dalpha': r'{\delta\boldsymbol{\alpha}}',
            'dbeta': r'{\delta\boldsymbol{\beta}}',
            'dgamma': r'{\delta\boldsymbol{\gamma}}',
            'dtheta': r'{\delta\boldsymbol{\theta}}',
            'dotdalpha': r'{\delta\dot{\boldsymbol{\alpha}}}',
            'dotdbeta': r'{\delta\dot{\boldsymbol{\beta}}}',
            'dotdgamma': r'{\delta\dot{\boldsymbol{\gamma}}}',
            'dotdtheta': r'{\delta\dot{\boldsymbol{\theta}}}',
            'dPos': r'{\dot{\Vec{r}}}',
            'dVel': r'{\dot{\Vec{v}}}',
            'angvel': r'{\boldsymbol{\omega}}',
            'gravity': r'{\Vec{g}_{W}}',
            'noise': r'{\Vec{n}}',
            'bias': r'{\Vec{b}}',
            'u': r'{\Vec{u}}',
            # Gyroscope
            'gyr': r'{\angvel}',
            'gyrMeas': r'{\angvel_{m}}',
            'gyrNoise': r'{\noise_{\omega}}',
            'gyrBias': r'{\bias_{\omega}}',
            'gyrBiasNoise': r'{\noise_{\bias_{\omega}}}',
            # Accelerometer
            'accMeas': r'{\acc_{m}}',
            'accNoise': r'{\noise_{a}}',
            'accBias': r'{\bias_{a}}',
            'accBiasNoise': r'{\noise_{\bias_{a}}}'
        }
    },
    'packages': ['base', 'ams', 'noerrors', 'noundefined'],
    'loader': {
        'load': ['[tex]/ams', '[tex]/noerrors', '[tex]/noundefined']
    }
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix =  {'.rst': 'restructuredtext'}

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = "en"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = None

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
html_theme_options = {
    "sticky_navigation": False,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``.
#
# html_sidebars = {}

# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'protodoc'

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'proto.tex', u'proto Documentation', u'Chris Choi', 'manual'),
]

# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [(master_doc, 'proto', u'proto Documentation', [author], 1)]

# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'proto', u'proto Documentation', author, 'proto',
     'One line description of project.', 'Miscellaneous'),
]

# -- Options for Epub output -------------------------------------------------

# Bibliographic Dublin Core info.
epub_title = project

# The unique identifier of the text. This can be a ISBN number
# or the project homepage.
#
# epub_identifier = ''

# A unique identification for the text.
#
# epub_uid = ''

# A list of files that should not be packed into the epub file.
epub_exclude_files = ['search.html']

# -- Extension configuration -------------------------------------------------
