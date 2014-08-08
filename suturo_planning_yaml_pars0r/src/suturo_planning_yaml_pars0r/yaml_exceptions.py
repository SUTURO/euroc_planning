class EmptyString(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class UnhandledValue(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class IllegalValue(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)