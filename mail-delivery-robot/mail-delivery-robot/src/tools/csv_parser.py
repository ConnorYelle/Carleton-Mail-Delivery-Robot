import os
import csv
import sys

CONFIG_DIR = sys.path[0] + "/config/"
BEACONS_FILE = "beacons.csv"
CONFIG_FILE = "config.csv"
BATTERY_CONFIG_FILE = "battery_config.csv" 

def parseCSVFile(filename, parse_numbers = False):
    '''
    Parses a CSV file located in te config directory and returns its contents as a dictionary.

    @param filename: The name of the csv file to parse.

    @return the contents of the parsed file.
    '''
    result = {}
    with open(CONFIG_DIR + filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        for row in reader:
            result[row[0]] = row[1] if not parse_numbers else float(row[1])
    return result

def loadBeacons():
    '''
    Loads all the beacons from the beacons file.
    '''
    return parseCSVFile(BEACONS_FILE)

def loadConfig():
    '''
    Loads the global config from the config file.
    '''
    return parseCSVFile(CONFIG_FILE, True)

def loadBatteryDockMapping():  
    '''
    Loads the battery dock mapping from the battery_config file.
    '''
    return parseCSVFile(BATTERY_CONFIG_FILE)