from distutils.core import setup

#This is a list of files to install, and where
#(relative to the 'root' dir, where setup.py is)
#You could be more specific.
files = ["smap_rllab/*"]

setup(name = "smap_rllab",
      version = "100",
      description = "Reinforcement Learning for Simultaneous Mapping And Planning (SMAP)",
      author = "Eric Heiden",
      author_email = "heiden@usc.edu",
      url = "whatever",
      #Name the folder where your packages live:
      #(If you have other packages (dirs) or modules (py files) then
      #put them into the package directory - they will be found
      #recursively.)
      packages = ['smap_rllab'],
      #'package' package must contain files (see list above)
      #I called the package 'package' thus cleverly confusing the whole issue...
      #This dict maps the package name =to=> directories
      #It says, package *needs* these files.
      package_data = {'package' : files },
      #'runner' is in the root.
      scripts = ["smap_rllab/trpo_smap.py"],
      long_description = """Really long text here."""
      #
      #This next part it for the Cheese Shop, look a little down the page.
      #classifiers = []
      )