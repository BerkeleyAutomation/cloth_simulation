"""
An implementation of a mouse class, that can be updated/modified to cut the cloth or disturb it. This can be used to interface with a physical or virtual mouse.
"""
class Mouse(object):

    def __init__(self, x=0, y=0, z=0, height_limit=False, down=False, button=0):
        self.down = down
        self.button = button
        self.x, self.y, self.z = x, y, z
        self.px, self.py, self.pz = x, y, z
        self.cut = 10
        self.influence = 5
        if height_limit:
            self.height_limit = height_limit
        else:
            self.height_limit = float('inf')

    def move(self, x, y):
        """
        Move mouse to a position on the canvas.
        """
        self.px, self.py = self.x, self.y
        self.x, self.y = x, y

    def clicked(self, event):
        """
        Handles click events of the mouse.
        """
        self.down = True

    def released(self, event):
        """
        Handles mouse release events.
        """
        self.down = False

    def moved(self, event):
        """
        Handles mouse move events.
        """
        self.move(event.xdata, event.ydata)