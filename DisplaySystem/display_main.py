import sys

from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.figure import Figure
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QTextEdit,
    QWidget,
    QHBoxLayout,
    QToolBar,
    QSlider,
    QMenuBar,
)


class ToolBar(QToolBar):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        scrollbar_action = QSlider()  
        # QAction("scrollbar", self)
        # scrollbar_action.setOrientation(QT)
        self.addWidget(scrollbar_action)


class ButtobBar(QVBoxLayout):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        vbox_widg = QWidget()

        vbox = QVBoxLayout()
        bttn = QPushButton()
        vbox.addWidget(QSlider())

        vbox_widg.setGeometry(bttn.pos().x(), bttn.height() + bttn.pos().y(), 100, 100)
        vbox_widg.setLayout(vbox)

        self.addWidget(bttn)
        self.addWidget(vbox_widg)


class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)


class Graph(QWidget):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        sc = MplCanvas(self, width=5, height=4, dpi=100)
        sc.axes.plot([0, 1, 2, 3, 4], [10, 1, 20, 3, 40])

        vbox = QVBoxLayout()
        vbox.addWidget(sc)
        self.setLayout(vbox)


# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        text = QTextEdit()
        text.setText("Hello World\n" * 50)
        # text.setDisabled(True)
        text.setReadOnly(True)

        self.setWindowTitle("My App")
        button = QPushButton("Press Me!")

        chart = Graph()

        hbox.addWidget(chart)
        hbox.addWidget(text)

        Buttob = ButtobBar()

        vbox.addLayout(Buttob)
        vbox.addLayout(hbox)
        vbox.addWidget(button)

        widget = QWidget()
        widget.setLayout(vbox)
        # Set the central widget of the Window.

        self.setCentralWidget(widget)


app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
