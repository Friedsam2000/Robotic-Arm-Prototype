# RT thesis template
Welcome to RT! We have prepared a template for you, which contains a practical folder structure for your project and LaTeX document templates for the exposé and the final report.

:warning: Please _do not distribute_ this repo nor make it public in _any_ way. It contains intellectual property of the Technische Universität München such as TUM colours, TUM logo and the logo of the Chair of Automatic Control.

## Document Templates
This repository contains two document templates:

* `/030_Expose`: Template for your exposé / concept paper (required for master's theses)
* `/900_Report`: Template for the final report / your thesis

Further details on how to use the templates are given below and in the templates themselves.

## LaTeX Template Quickstart
### Step 1: Get LaTeX
#### For Windows: MikTeX
* Download and install [MiKTeX](http://miktex.org/download)
* Make sure to select "Install MiKTeX only for me" and *not* "for anyone" to avoid problems with updates and configurations.
#### For Linux: TeXLive
* Follow the installation instructions for example [here](https://linuxhint.com/install-latex-ubuntu/)

### Step 2: Set up TeXstudio
* Get [TeXstudio](http://www.texstudio.org/).
* Make sure the following settings are correct:
	* Go to Options / Configure TeXstudio.
	* In the Build section make sure that PdfLaTeX is chosen as the default compiler and Biber as the default bibliography tool.
	* Click 'Compile & View' or F5 to create the document.


## Tools
Good tools are important for your workflow. Here are some recommended programs:
* Bibliography tool, for example [Citavi](https://www.citavi.com/de)
    * Be sure to enable LaTeX support in Citavi (options/Citation)
* Text editors:
    * [Notepad++](https://notepad-plus-plus.org/)
    * [Atom](https://atom.io/) (also install packages `platformio-ide-terminal`, `zen`, `scroll-sync`, `language-latex`, `language-matlab-octave`)
* Git clients:
    * [SmartGit](http://www.syntevo.com/smartgit/) (great functionality, ideal for power users)
    * [SourceTree](https://www.sourcetreeapp.com/)
    * [GitHub Desktop](https://desktop.github.com/) (nice and simple interface, less low-level control, ideal for beginners)
* [TikzEdt](http://www.tikzedt.org/)
* [Detexify](https://detexify.kirelabs.org/classify.html) (Gives LaTeX code for recognized handwritten symbols)

Good luck and have fun! :rocket:
