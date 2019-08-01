import os
from urllib.request import urlretrieve

files = ["replay.txt", "roadnet.json"]
repo_url = "https://github.com/cityflow-project/data/raw/master/frontend/replay/"

folder = "replay"
if __name__ == '__main__':
    if not (os.path.exists(folder) and os.path.isdir(folder)):
        os.mkdir(folder)
        print("make %s folder, done!" % folder)
    
    for file_name in files:
        if os.path.exists(folder + "/" + file_name):
            print(file_name + " found.")
        else:
            print("retrieving %s from cityflow-project/data/frontend/replay" % file_name)
            urlretrieve(repo_url + file_name, folder + "/" + file_name)
            print("done!")