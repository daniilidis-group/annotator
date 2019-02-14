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

class ZoomGraphicsView(QGraphicsView):
    def __init__ (self, parent=None):
        super(ZoomGraphicsView, self).__init__ (parent)
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

class AnnotationGraphicsView(ZoomGraphicsView):
    def __init__ (self, listener, pixmap, parent=None):
        super(AnnotationGraphicsView, self).__init__ (parent)
        self.pixmap = pixmap
        # the rubber band is for drawing the rectangle
        # while the mouse is pressed down.
        self.rubberBand = QRubberBand(QRubberBand.Rectangle, self)
        self.setMouseTracking(True)
        self.origin = QPoint()
        self.changeRubberBand = False
        # list of rectangles created
        self.rectangles = []
        # top left corner of current rectangle
        self.topLeft = QPoint(-1, -1)

    def makeRect(self, rect):
        r = QGraphicsRectItem(rect, self.pixmap)
        r.setPen(QPen(Qt.green, 3))
        r.setFlag(QGraphicsItem.ItemIsMovable, False)
        r.setFlag(QGraphicsItem.ItemIsSelectable, False)
        return (r)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            # star the rubber band
            self.origin = event.pos()
            self.rubberBand.setGeometry(QRect(self.origin, QSize()))
            self.rubberBand.show()
            self.changeRubberBand = True
            
            if self.topLeft == QPoint(-1, -1):
                self.topLeft = self.mapToScene(event.pos())
        if event.button() == Qt.RightButton:
            spos = self.mapToScene(event.pos())
            d = 8 # number of pixels in scene
            # pick rectangles close to mouse position
            to_remove = [r for r in self.rectangles if
                         r.rect().adjusted(-d, -d, d, d).contains(spos)
                         and  not r.rect().adjusted(d, d, -d, -d).contains(spos)]
            map(lambda x:self.scene().removeItem(x), to_remove)
            self.rectangles = [x for x in self.rectangles if x not in to_remove]
            print 'rectangles removed: ', len(to_remove)
        QGraphicsView.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.changeRubberBand:
            self.rubberBand.setGeometry(QRect(self.origin, event.pos()).normalized())
        QGraphicsView.mouseMoveEvent(self, event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.changeRubberBand = False
            self.rubberBand.hide()
            if not self.topLeft == QPoint(-1, -1):
                r = QRectF(self.topLeft, self.mapToScene(event.pos()))
                self.rectangles.append(self.makeRect(r))
                self.topLeft = QPoint(-1, -1)

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
        self.scene = QGraphicsScene(self)
        self.p_item = self.scene.addPixmap(QPixmap("foo.png"))
        self.gv = AnnotationGraphicsView(self, self.p_item)
        self.gv.setScene(self.scene) 
        self.gv.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.gv.fitInView(self.scene.sceneRect())
        self.img = None
        lay = QHBoxLayout(self)
        lay.addWidget(self.gv)
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

    def done(self):
        if not self.img:
            print 'no image received yet'
            return
        rl = RectList()
        rl.header = self.img.header
        for qri in self.gv.rectangles:
            qr = qri.rect()
            r = Rect(x=qr.topLeft().x(), y=qr.topLeft().y(), width=qr.width(), height=qr.height())
            rl.rectangles.append(r)
        print self.topic, ' publish rects: ', len(rl.rectangles)
        self.rectPub.publish(rl)
    
        
class MainWidget(QWidget):
    def __init__(self, topics):
        QWidget.__init__(self, None)
        print "starting ros worker thread!"
        self.views = [ViewWidget(self, t) for t in topics]
        self.rosworker = RosWorker(main_gui = self, views = self.views)
        self.rosworker.start()
        self.viewsFrame = QFrame(self)
        vlay = QGridLayout(self.viewsFrame)
        ncols = 4
        for i in range(0, len(self.views)):
            vlay.addWidget(self.views[i], i/ncols, i % ncols)
        self.buttonsFrame = QFrame(self)
        blay = QHBoxLayout(self.buttonsFrame)
        
        mlay = QVBoxLayout(self)
        mlay.addWidget(self.viewsFrame)
        mlay.addWidget(self.buttonsFrame)
        
        self.doneButton = QPushButton('Done', self.buttonsFrame)
        self.quitButton = QPushButton('Quit', self.buttonsFrame)
        blay.addWidget(self.doneButton)
        blay.addWidget(self.quitButton)
        
        self.doneButton.clicked.connect(self.handleDone)
        self.quitButton.clicked.connect(self.close)

    def handleDone(self, tmp):
        print "done!"
        for v in self.views:
            v.done()

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
