import numpy
from suturo_environment_msgs.msg import Cell as CellMessage

__author__ = 'ichumuh'


class Cell(CellMessage):
    Free = 0
    Unknown = 1
    Obstacle = 2
    Object = 3

    BLUE = 255
    GREEN = 65280
    CYAN = 65535
    RED = 16711680
    MAGENTA = 16711935
    YELLOW = 16776960
    UNDEF = -1

    BLUE_ID = 0
    GREEN_ID = 1
    CYAN_ID = 2
    RED_ID = 3
    MAGENTA_ID = 4
    YELLOW_ID = 5
    UNDEF_ID = 6

    BLUE_HEX = '0000ff'
    GREEN_HEX = '00ff00'
    CYAN_HEX = '00ffff'
    RED_HEX = 'ff0000'
    MAGENTA_HEX = 'ff00ff'
    YELLOW_HEX = 'ffff00'

    #point under min_z_value are equal to z=0
    min_z_value = 0.01

    #a cell only gets a color, when at least 5% of the points have this color
    undef_threshold = 0.05

    def __init__(self, average_z=0, highest_z=0, marked=False, state=None,
                        points=None):
        if state is None:
            state = Cell.Unknown
        if points is None:
            points = numpy.asarray([0 for x in range(7)], dtype=numpy.int)
        super(Cell, self).__init__(average_z=average_z, highest_z=highest_z, marked=marked, state=state, points=points)
        self.segment_id = 0
        self.threshold_min_points = 15

    def __str__(self):
        return "free: " + str(self.is_free()) + \
               " obstacle: " + str(self.is_obstacle()) + \
               " unknown: " + str(self.is_unknown()) + \
               " Points: " + str(self.get_num_points()) + \
               " segment_id: " + str(self.segment_id) + \
               "z: " + str(self.average_z) + \
               "max z: " + str(self.highest_z) + "\n"

    def __eq__(self, other):
        return other.is_obstacle() and self.is_obstacle() or \
               other.is_free() and self.is_free() or \
               other.is_unknown() and self.is_unknown()

    def to_msg(self):
        return self

    def update_cell(self, z, color):
        """
        Updates a cell.
        :param z: z value of a point in this cell
        :type: float
        :param color: color of a point in this cell
        :type: float
        """
        z2 = self.average_z * self.get_num_points()
        if z > self.min_z_value:
            if color == self.BLUE:
                self.points[self.BLUE_ID] += 1
            elif color == self.GREEN:
                self.points[self.GREEN_ID] += 1
            elif color == self.CYAN:
                self.points[self.CYAN_ID] += 1
            elif color == self.RED:
                self.points[self.RED_ID] += 1
            elif color == self.MAGENTA:
                self.points[self.MAGENTA_ID] += 1
            elif color == self.YELLOW:
                self.points[self.YELLOW_ID] += 1
            else:
                self.points[self.UNDEF_ID] += 1
        else:
            self.points[self.UNDEF_ID] += 1

        self.average_z = (z2 + z) / self.get_num_points()
        if z > self.highest_z:
            self.highest_z = z
        self._update_state()

    def _update_state(self):
        """
        Updates the state of the cell depending on the average z value
        """
        if not self.is_object() and self._enough_points():
            if self.average_z <= self.min_z_value:
                self.state = self.Free
            else:
                self.state = self.Obstacle

    def _enough_points(self):
        """
        :return: True, if this cell has enough points to be an obstacle/object
        :type: bool
        """
        return self.get_num_points() > self.threshold_min_points

    def get_num_points(self):
        """
        :return: number of points in the cell
        :type: int
        """
        return sum(self.points)

    def id_to_color(self, id):
        """
        :param id: color id
        :type: int
        :return: corresponding color
        :type: int
        """
        if id == self.BLUE_ID:
            return self.BLUE
        elif id == self.GREEN_ID:
            return self.GREEN
        elif id == self.CYAN_ID:
            return self.CYAN
        elif id == self.RED_ID:
            return self.RED
        elif id == self.MAGENTA_ID:
            return self.MAGENTA
        elif id == self.YELLOW_ID:
            return self.YELLOW
        return self.UNDEF

    def color_to_id(self, id):
        """
        :param id: color
        :type: int
        :return: corresponding color id
        :type: int
        """
        if id == self.BLUE:
            return self.BLUE_ID
        elif id == self.GREEN:
            return self.GREEN_ID
        elif id == self.CYAN:
            return self.CYAN_ID
        elif id == self.RED:
            return self.RED_ID
        elif id == self.MAGENTA:
            return self.MAGENTA_ID
        elif id == self.YELLOW:
            return self.YELLOW_ID
        else:
            return self.UNDEF

    def color_id_to_hex(self, color_id):
        if color_id == self.BLUE_ID:
            return self.BLUE_HEX
        elif color_id == self.GREEN_ID:
            return self.GREEN_HEX
        elif color_id == self.CYAN_ID:
            return self.CYAN_HEX
        elif color_id == self.RED_ID:
            return self.RED_HEX
        elif color_id == self.MAGENTA_ID:
            return self.MAGENTA_HEX
        elif color_id == self.YELLOW_ID:
            return self.YELLOW_HEX
        return None

    # setter

    def set_free(self):
        """
        Sets the state of the cell to free. Does not change any other inner attributes.
        """
        self.state = self.Free

    def set_object(self, is_object=True):
        """
        Sets the state of the cell to object. Does not change any other inner attributes.
        """
        self.state = self.Object

    def set_obstacle(self):
        """
        Sets the state of the cell to obstacle. Does not change any other inner attributes.
        """
        self.state = self.Obstacle

    def set_unknown(self):
        """
        Sets the state of the cell to unknown. Does not change any other inner attributes.
        """
        self.state = self.Unknown

    def set_mark(self, marked=True):
        """
        Marks a cell, which will make it appear orange. No other side effects
        :param marked: bool
        """
        self.marked = marked

    def set_blue(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.BLUE_ID)

    def set_green(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.GREEN_ID)

    def set_red(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.RED_ID)

    def set_yellow(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.YELLOW_ID)

    def set_cyan(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.CYAN_ID)

    def set_magenta(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.MAGENTA_ID)

    def set_undef(self):
        """
        Changes the amount of colored points, to change its color.
        """
        self.set_color(self.UNDEF_ID)

    def set_color(self, color_id):
        """
        Changes the amount of colored points, to change its color.
        :param color_id:
        :type: int
        """
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[color_id] = 50


    #getter

    def get_state(self):
        """
        :return: the state of the cell
        :type: int
        """
        return self.state

    def is_object(self):
        return self.get_state() == self.Object

    def is_free(self):
        return self.get_state() == self.Free

    def is_obstacle(self):
        return self.get_state() == self.Obstacle

    def is_unknown(self):
        return self.get_state() == self.Unknown

    def is_marked(self):
        return self.marked

    def get_color(self):
        return self.id_to_color(self.get_color_id())

    def get_color_hex(self):
        return self.color_id_to_hex(self.get_color_id())

    def get_color_id(self):
        """
        :return: the color id of the cell
        :type: int
        """
        if self.get_num_points() == 0:
            return self.UNDEF_ID
        id = -1
        for i in xrange(len(self.points) - 1):
            if ((self.points[i] + 0.0) / (self.get_num_points() + 0.0) > self.undef_threshold and id == -1) or self.points[
                i] > self.points[id]:
                id = i
        if id == -1:
            return self.UNDEF_ID
        return id

    def is_blue(self):
        return self.get_color_id() == self.BLUE_ID and (self.is_obstacle() or self.is_object())

    def is_green(self):
        return self.get_color_id() == self.GREEN_ID and (self.is_obstacle() or self.is_object())

    def is_red(self):
        return self.get_color_id() == self.RED_ID and (self.is_obstacle() or self.is_object())

    def is_cyan(self):
        return self.get_color_id() == self.CYAN_ID and (self.is_obstacle() or self.is_object())

    def is_magenta(self):
        return self.get_color_id() == self.MAGENTA_ID and (self.is_obstacle() or self.is_object())

    def is_yellow(self):
        return self.get_color_id() == self.YELLOW_ID and (self.is_obstacle() or self.is_object())

    def is_undef(self):
        return self.get_color_id() == self.UNDEF_ID and (self.is_obstacle() or self.is_object())
