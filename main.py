import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from ui.mainwindow import MainWindow

if __name__ == "__main__":
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Shingleback Ground Control")
    app.setStyle("Fusion")
    app.setFont(QtGui.QFont("DejaVu Sans", 11))
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
