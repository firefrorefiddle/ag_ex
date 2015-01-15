#!/usr/bin/python

'''
Created on Jan 11, 2015

@author: smortezapoor
'''
import sys
from Tools import Helper


def main(argv):
    sys.path.insert(1,'/usr/local/bin/')
    if len(argv) > 1:
        fileLocation = argv[1]
    else:
        fileLocation = '../../instances/tcbvrp_180_1_T720_m10.prob'
    g = Helper.Parse(fileLocation)
    Helper.ShowIt(g)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
