import numpy


class SearchGrid:

    _fields = None

    def __init__(self, num_of_fields_x, num_of_fields_y):
        self._fields = numpy.zeros((num_of_fields_x, num_of_fields_y))

    def get_field(self, x, y):
        return self._fields[x][y]

    def set_field(self, x, y, value):
        self._fields[x][y] = value
        return value