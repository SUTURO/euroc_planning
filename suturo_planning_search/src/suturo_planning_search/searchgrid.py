class SearchGrid:

    field = []

    def __init__(self, num_of_fields_x, num_of_fields_y):
        for x in range(0, num_of_fields_x):
            row = []
            for y in range(0, num_of_fields_y):
                row.append(SearchField(x, y))
            self.field.append(row)


class SearchField:

    # True if the field is blocked by an obstacle
    blocked = False

    # Value of the field. Increases if the field gets seen.
    value = 0

    # x position
    x = 0

    # y position
    y = 0

    def __init__(self, x, y, blocked=False, value=0):
        self.x = x
        self.y = y
        self.blocked = blocked
        self.value = value