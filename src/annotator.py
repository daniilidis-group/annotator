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
    def __init__(self, main_gui, views):
        QThread.__init__(self, parent=None)
        self.main_gui = main_gui
        self.views = views
    def run(self):
        def __del__(self):
            self.wait()
        rospy.init_node('annotator', disable_signals=True)
        for v in self.views:
            v.sub = rospy.Subscriber(v.topic, Image, v.img_callback);
        print "finished subscribing to img, spinning now"
        rospy.spin()


class ViewWidget(QWidget):
    signal = pyqtSignal(QImage)
    def __init__(self, parent=None, topic = ""):
        QWidget.__init__(self, parent)
        self.bridge = CvBridge()
        self.topic = topic
        self.gv = ZoomQGraphicsView()
        self.scene = QGraphicsScene(self)
        self.gv.setScene(self.scene) 
        self.gv.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.gv.fitInView(self.scene.sceneRect())
        lay = QHBoxLayout(self)
        lay.addWidget(self.gv)
        self.p_item = self.scene.addPixmap(QPixmap("foo.png"))
        self.signal.connect(self.handle_img_update)

    def handle_img_update(self, img):
        self.p_item.setPixmap(QPixmap.fromImage(img))

    def img_callback(self, img):
        print self.topic, ' got image message: '
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        q_img = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
        self.signal.emit(q_img)
    
        
class MainWidget(QWidget):
    def __init__(self, topics):
        QWidget.__init__(self, None)
        print "starting ros worker thread!"
        self.views = [ViewWidget(self, t) for t in topics]
        self.rosworker = RosWorker(main_gui = self, views = self.views)
        self.rosworker.start()
        lay = QGridLayout(self)
        ncols = 2
        for i in range(0, len(self.views)):
            lay.addWidget(self.views[i], i/ncols, i % ncols)


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
    w = MainWidget(["/cam_sync/cam0/image_raw",
                    "/cam_sync/cam1/image_raw",
                    "/cam_sync/cam2/image_raw",
                    "/cam_sync/cam3/image_raw"
    ])
    w.show()
    sys.exit(app.exec_())
