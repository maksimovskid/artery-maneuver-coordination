# Build this project's documentation using MkDocs

The checked-in `mkdocs.yml` builds documentation for this
[research repository](https://github.com/maksimovskid/artery-maneuver-coordination).
The upstream framework website at [artery.v2x-research.eu](http://artery.v2x-research.eu)
provides background on Artery; it is not the documentation deployment of this fork.
Building locally does not publish a website.

## Installing Dependencies
[PEP 668](https://peps.python.org/pep-0668/) recommends to install Python dependencies in a virtual environment.
We propose the following steps when in the root directory:
```console
python3 -m venv venv
venv/bin/pip install -r docs/requirements.txt
```

## Build or Serve
You can build the website from the root directory where *mkdocs.yml* is located by calling:
```console
venv/bin/mkdocs build
```
This will generate the website in the *www* directory.

Alternatively, you can start a temporary webserver delivering the website locally with:
```console
venv/bin/mkdocs serve
```
