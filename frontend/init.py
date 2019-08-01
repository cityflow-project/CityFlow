import os
from urllib.request import urlretrieve

libraries = ["axios.min.js", "bootstrap.min.css", "pixi.min.js", "spinner.css", "viewport.js"]
repo_url = "https://github.com/cityflow-project/data/raw/master/frontend/library/"

folder = "library"
if __name__ == '__main__':
    if not (os.path.exists(folder) and os.path.isdir(folder)):
        os.mkdir(folder)
        print("make %s folder, done!" % folder)
    
    for library in libraries:
        if os.path.exists(folder + "/" + library):
            print(library + " found.")
        else:
            print("retrieving %s from cityflow-project/data/frontend/library" % library)
            urlretrieve(repo_url + library, folder + "/" + library)
            print("done!")