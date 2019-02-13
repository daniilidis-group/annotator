#!/usr/bin/python
#
# Bernd Pfrommer 2019
#
# image annotation GUI
# 
#
import sys
import roslib
import rospy

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class ZoomQGraphicsView(QGraphicsView):
    def __init__ (self, parent=None):
        super(ZoomQGraphicsView, self).__init__ (parent)

    def wheelEvent(self, event):
        # Zoom Factor
        zoomInFactor = 1.25
        zoomOutFactor = 1 / zoomInFactor

        # Set Anchors
        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)

        # Save the scene pos
        oldPos = self.mapToScene(event.pos())

        # Zoom
        if event.delta() > 0:
            zoomFactor = zoomInFactor
        else:
            zoomFactor = zoomOutFactor
        self.scale(zoomFactor, zoomFactor)

        # Get the new position
        newPos = self.mapToScene(event.pos())

        # Move scene to old position
        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())

class RosWorker(QThread):
    signal = pyqtSignal(QImage)
    def __init__(self, app):
        QThread.__init__(self, parent=None)
        self.bridge = CvBridge()
        self.app = app
    def run(self):
        def __del__(self):
            self.wait()
        rospy.init_node('annotator', disable_signals=True)
        self.sub = rospy.Subscriber('/cam_sync/cam0/image_raw', Image, self.img_callback);
        print "finished subscribing to img, spinning now"
        self.signal.connect(self.app.handle_img_update)
        rospy.spin()

    def img_callback(self, img):
        print 'got image message: ', img.header
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        q_img = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
        self.signal.emit(q_img)
        print 'done updating image!'

class Widget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        print "starting ros worker thread!"
        self.rosworker = RosWorker(app = self)
        self.rosworker.start()
        self.btn = QPushButton("Add Line")
        self.gv = ZoomQGraphicsView()
        self.scene = QGraphicsScene(self)
        self.gv.setScene(self.scene) 
        self.gv.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.gv.fitInView(self.scene.sceneRect())

        self.scene2 = QGraphicsScene(self)
        self.gv2 = ZoomQGraphicsView()
        self.gv2.setScene(self.scene2) 
        self.gv2.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.gv2.fitInView(self.scene2.sceneRect())
                
        lay = QHBoxLayout(self)
        lay.addWidget(self.btn)
        lay.addWidget(self.gv)
        lay.addWidget(self.gv2)

        print ("adding lena!")
        self.p_item1 = self.scene.addPixmap(QPixmap("foo.png"))
        self.p_item2 = self.scene2.addPixmap(QPixmap("foo.png"))
        self.btn.clicked.connect(self.add_line)


    def handle_img_update(self, img):
        self.p_item1.setPixmap(QPixmap.fromImage(img))


    def add_line(self):
        p1 = self.p_item1.boundingRect().topLeft()
        p2 = self.p_item1.boundingRect().center()
        line = QGraphicsLineItem(QLineF(p1, p2), self.p_item1)
        line.setPen(QPen(Qt.red, 5))
        line.setFlag(QGraphicsItem.ItemIsMovable, True)
        line.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.gv.fitInView(self.scene.sceneRect())
        self.gv2.fitInView(self.scene2.sceneRect())


if __name__ == '__main__':
    print ("starting up")
    app = QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(app.exec_())
