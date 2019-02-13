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
import ctypes

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from bird_recording.msg import RectList
from bird_recording.msg import Rect

class ZoomQGraphicsView(QGraphicsView):
    rectChanged = pyqtSignal(QRect)
    def __init__ (self, listener, parent=None):
        super(ZoomQGraphicsView, self).__init__ (parent)
        self.rectChanged.connect(listener.rect_callback)

        self.rubberBand = QRubberBand(QRubberBand.Rectangle, self)
        self.setMouseTracking(True)
        self.origin = QPoint()
        self.changeRubberBand = False

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

    def mousePressEvent(self, event):
        self.origin = event.pos()
        self.rubberBand.setGeometry(QRect(self.origin, QSize()))
        self.rectChanged.emit(self.rubberBand.geometry())
        self.rubberBand.show()
        self.changeRubberBand = True
        QGraphicsView.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.changeRubberBand:
            self.rubberBand.setGeometry(QRect(self.origin, event.pos()).normalized())
            self.rectChanged.emit(self.rubberBand.geometry())
        QGraphicsView.mouseMoveEvent(self, event)

    def mouseReleaseEvent(self, event):
        self.changeRubberBand = False
        QGraphicsView.mouseReleaseEvent(self, event)

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
            v.rectPub = rospy.Publisher(v.topic + "/rect", RectList, queue_size=1)
            v.maskPub = rospy.Publisher(v.topic + "/mask", Image, queue_size=1)
        print "finished subscribing to img, spinning now"
        rospy.spin()


class ViewWidget(QWidget):
    signal = pyqtSignal(QImage)
    def __init__(self, parent=None, topic = ""):
        QWidget.__init__(self, parent)
        self.bridge = CvBridge()
        self.topic = topic
        self.gv = ZoomQGraphicsView(self)
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
        self.gv.fitInView(self.scene.sceneRect())

    def img_callback(self, img):
        self.img = img
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except CvBridgeError as e:
            print(e)
            return
        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        q_img = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
        self.signal.emit(q_img)

    def rect_callback(self, qr):
        print self.topic, " got rect!", str(qr)
        rl = RectList()
        rl.header = self.img.header
        r = Rect(x=qr.topLeft().x(), y=qr.topLeft().y(), width=qr.width(), height=qr.height())
        print str(r)
        rl.rectangles.append(r)
        self.rectPub.publish(rl)
    
        
class MainWidget(QWidget):
    def __init__(self, topics):
        QWidget.__init__(self, None)
        print "starting ros worker thread!"
        self.views = [ViewWidget(self, t) for t in topics]
        self.rosworker = RosWorker(main_gui = self, views = self.views)
        self.rosworker.start()
        lay = QGridLayout(self)
        ncols = 4
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
                    "/cam_sync/cam3/image_raw",
                    "/cam_sync/cam4/image_raw"
    ])
    w.show()
    sys.exit(app.exec_())
