# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Benji'
copyright = '2025, Cave in the Mountains'
author = 'Cave in the Mountains'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []
extensions = [
    "sphinx_rtd_theme",
    "sphinx.ext.todo",
    "sphinx.ext.extlinks",
    "sphinx.ext.autodoc",
    "sphinx.ext.graphviz",
    "sphinxcontrib.jquery",
    "sphinxcontrib.programoutput",
    "myst_parser",
    "nbsphinx",
]

myst_enable_extensions = [
    "dollarmath",
    "substitution",
]

# Allow MyST to include files from parent directory (for README.md)
myst_heading_anchors = 3

# Substitutions for conditional content
myst_substitutions = {
    "badge": "[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://sailorbob134280.github.io/benji/)",
}

# -- Options for nbsphinx ---------------------------------------------------
# Execute notebooks during build to ensure outputs are current.
# nbsphinx_execute = 'auto'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'gnc/notebooks/appendix', 'overview-pres']

source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'restructuredtext',
    '.md': 'markdown',
}

pygments_style = "sphinx"
highlight_language = "none"

todo_include_todos = False

nitpick_ignore = [
    # ignore C standard identifiers (they are not defined in Zephyr docs)
    ("c:identifier", "FILE"),
    ("c:identifier", "int8_t"),
    ("c:identifier", "int16_t"),
    ("c:identifier", "int32_t"),
    ("c:identifier", "int64_t"),
    ("c:identifier", "intptr_t"),
    ("c:identifier", "off_t"),
    ("c:identifier", "size_t"),
    ("c:identifier", "ssize_t"),
    ("c:identifier", "time_t"),
    ("c:identifier", "uint8_t"),
    ("c:identifier", "uint16_t"),
    ("c:identifier", "uint32_t"),
    ("c:identifier", "uint64_t"),
    ("c:identifier", "uintptr_t"),
    ("c:identifier", "va_list"),
]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "logo_only": True,
    "prev_next_buttons_location": None,
    "navigation_depth": 5,
}
html_title = "Benji Documentation"
html_logo = "_static/images/logo.png"
html_favicon = "_static/images/logo.png"
html_static_path = ['_static']
html_extra_path = ['_static/.nojekyll', 'overview-pres/dist']
html_last_updated_fmt = "%b %d, %Y"
html_domain_indices = False
html_show_sourcelink = False
html_show_sphinx = False

html_context = {
    "reference_links": {
        "GitHub Repository": "https://github.com/sailorbob134280/benji",
        "Overview Presentation": "presentation/index.html",
    },
}

# -- Options for LaTeX/PDF output ----------------------------------------
# Use xelatex for Unicode support (Greek letters in notebooks)
latex_engine = 'xelatex'

latex_logo = '_static/images/logo.png'

latex_documents = [
    ('index', 'benji.tex', 'Benji Documentation',
     'Cave in the Mountains', 'manual'),
]

# Suppress missing image warnings/errors for remote images (e.g., shields.io badges)
suppress_warnings = ['image.not_readable']

# -- Options for sphinx-sitemap ----------------------------------------

sitemap_url_scheme = "{link}"

def setup(app):
    app.add_css_file("css/custom.css")
    app.add_js_file("js/custom.js", type="module")
