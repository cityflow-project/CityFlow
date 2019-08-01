import os
from urllib.request import urlretrieve

libraries = ["axios.min.js", "bootstrap.min.css", "pixi.min.js", "spinner.css", "viewport.js"]
repo_url = "https://github.com/cityflow-project/data/raw/master/frontend/library/"

if __name__ == '__main__':
    if not (os.path.exists("library") and os.path.isdir("library")):
        os.mkdir("library")
        print("make library folder, done!")
    
    for library in libraries:
        if os.path.exists("library/" + library):
            print(library + " found.")
        else:
            print("retrieving %s from cityflow-project/data/frontend/library" % library)
            urlretrieve(repo_url + library, "library/" + library)
            print("done!")