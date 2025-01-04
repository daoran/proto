#!/usr/bin/env python3
from jinja2 import Environment, FileSystemLoader
from livereload import Server
server = Server()
server.serve(root='.')
