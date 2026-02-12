from tools.nav_parser import loadMap

class Map:

    '''
    Defines the hashmap that the robot uses for its routes.
    Remember: the map has a hashmap within a hashmap as values. Therefore, you would retrieve a value from within the map as follows:

    routing_map["Loeb1"]["Southam"] -> returns U-TURN
    '''

    def __init__(self):
        '''
        Constructor for the map. Essentially initializes the map into a hashmap.
        '''
        self.routing_map = loadMap()

    def getMap(self):
        '''
        Typical getter for the whole routing map.
        '''
        return self.routing_map

    def getDirection(self, source, destination):
        '''
        General method to return the value from a given key in a key-value pair.

        @param source: the source of the trip.
        @param destination: the destination of the trip.
        '''
        return self.routing_map[source][destination]

    def exists(self, location):
        '''
        Checks to see if the location exists in the map.

        @param location: the building to verify.
        '''
        return location in self.routing_map
