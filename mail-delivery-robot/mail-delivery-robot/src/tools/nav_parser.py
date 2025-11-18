import os
import csv
import sys

CONFIG_DIR = sys.path[0] + "/config/"

ORIENTATION_FILE = "beacon_connections.csv"
MAP_FILE = "map.csv"

def parseCSVFile(filename) -> dict:
    '''
    Parses a CSV file (related to navigation) in its config directory and returns a dictionary of contents.

    This is slightly different from the other parser, as this one deals with multiple columns. 

    @param filename: Name of the CSV file to parse.
    @return A dictonary of the contents of the parsed file.

    '''
    result = {}
    with open(CONFIG_DIR + filename) as CSVFile:
        reader = csv.reader(CSVFile, delimiter=",")
        firstRow = []
        for row in reader:
            if firstRow == []:
                firstRow = row[1:]
            else:
                rowDict = {firstRow[i]: row[i+1] for i in range(len(row)-1)}
                result[row[0]] = rowDict
        return result

def loadConnections():
    '''
    Loads beacon connections from ORIENTATION_FILE.
    '''
    return parseCSVFile(ORIENTATION_FILE)

def loadMap():
    '''
    Load the map from MAP_FILE.
    '''
    return parseCSVFile(MAP_FILE)

