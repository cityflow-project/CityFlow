import unittest
import os
import json
from tools.converter.osm2cityflow import extract, draw


class TestOsmConverter(unittest.TestCase):
    dataPath = os.getcwd() + '/data/tools/Converter/examples/'
    osmFile = dataPath + 'statecollege.osm'
    CityFlowFile = dataPath + 'statecollege_roadnet.json'

    def test_converter(self):
        nodes = extract(osmFile=self.osmFile)
        draw(nodes, self.CityFlowFile, True)
        with open(self.CityFlowFile, 'r') as f:
            roadnet = json.load(f)
        intersectionsNum = len(roadnet["intersections"])
        roadsNum = len(roadnet["roads"])
        self.assertTrue(intersectionsNum <= roadsNum)


if __name__ == '__main__':
    unittest.main(verbosity=2)
