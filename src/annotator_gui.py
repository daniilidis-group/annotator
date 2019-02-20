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
import cv2
import numpy as np
from annotator.srv import *
from flex_sync import Sync

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from bird_recording.msg import RectList
from bird_recording.msg import Rect

class ZoomGraphicsView(QGraphicsView):
    """ The ZoomGraphicssView is regular QGraphicsView, but
    with the wheel events used to zoom in/out """
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
    """ AnnotationGraphicsView has the logic for rectangle markup:
    - left button hold and drag creates rectangle
    - right mouse button click on rectangle erases it
    """

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
                         r.rect().adjusted(-d, -d, d, d).contains(spos)]
            map(lambda x:self.scene().removeItem(x), to_remove)
            self.rectangles = [x for x in self.rectangles
                               if x not in to_remove]
        QGraphicsView.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.changeRubberBand:
            self.rubberBand.setGeometry(QRect(self.origin, event.pos()).normalized())
        QGraphicsView.mouseMoveEvent(self, event)

    def clearRectangles(self):
        map(lambda x:self.scene().removeItem(x), self.rectangles)
        self.rectangles = []

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
    """ The RosWorker is a thread that services the ROS event loop.
    Careful when interacting with the main GUI loop, you need to either lock
    (and face the risk of deadlock), or emit signals when interacting with
    the main GUI thread"""
    def __init__(self, main_gui, views):
        QThread.__init__(self, parent=None)
        self.main_gui = main_gui
        self.views = views
    def __del__(self):
        self.wait()
    def run(self):
        """ entry point for the ROS worker thread """
        #
        # do all the subscriptions and ROS based stuff in this thread
        #
        
        rospy.init_node('annotator', disable_signals=True)
        #
        # the sync will be directly called by each view. Once an image
        # is complete, the sync will call syncCallback
        #
        self.sync = Sync([v.topic for v in self.views], self.syncCallback)
        for v in self.views:
            v.sub = rospy.Subscriber(v.topic, Image, v.img_callback, queue_size = 10)
            v.rectPub = rospy.Publisher(v.topic + "/rect", RectList, queue_size = 2)
            v.maskPub = rospy.Publisher(v.topic + "/mask", Image, queue_size = 2)
            v.sync    = self.sync
        svc = '/bag_player/command'
        rospy.loginfo("looking for rosbag_player node service as " + svc)
        rospy.wait_for_service(svc)
        try:
            self.playerCommand = rospy.ServiceProxy(svc, PlayerCmd)
            rospy.loginfo('found service ' + svc)
        except rospy.ServiceException as e:
            rospy.logerr('cannot connect to service: ' + str(e))
            return

        rospy.loginfo("finished subscribing to img, spinning now")
        rospy.spin() # this call never returns

    def resetSync(self):
        self.sync.reset()
    
    def syncCallback(self, msgvec):
        rospy.loginfo('got sync callback for time: %.3f' % msgvec[0].header.stamp.to_sec())
        for i in range(0, len(self.views)):
            self.views[i].sync_img_callback(msgvec[i])

class ViewWidget(QWidget):
    signal = pyqtSignal(QImage)
    timeChanged = pyqtSignal(genpy.rostime.Time)
    def __init__(self, parent=None, topic = "", is_monitor = False):
        QWidget.__init__(self, parent)
        self.mainGui   = parent
        self.isMonitor = is_monitor
        self.bridge = CvBridge()
        self.topic = topic
        self.scene = QGraphicsScene(self)
        self.p_item = self.scene.addPixmap(QPixmap("foo.png"))
        self.gv = AnnotationGraphicsView(self, self.p_item)
        self.gv.setScene(self.scene) 
        self.gv.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.gv.fitInView(self.scene.sceneRect())
        #
        self.img_msg = None
        self.state = "playing"
        self.sync = None
        lay = QHBoxLayout(self)
        lay.addWidget(self.gv)
        
        # connect image update to myself. This is
        # to separate the work so the image update is done
        # in the gui thread, not the ROS thread.
        self.signal.connect(self.handle_img_update)
        
        # connect timeChanged to the main gui, same
        # reason as above
        self.timeChanged.connect(self.mainGui.handleTimeChanged)

    def setState(self, s):
        self.state = s
        if self.state == 'playing' and not self.isMonitor:
            self.hide()
        if self.state == 'replaying':
            self.mainGui.rosworker.resetSync()
            self.show()

        
    def handle_img_update(self, img):
        """ handle_img_update is run by the GUI thread """
        self.p_item.setPixmap(QPixmap.fromImage(img))
        self.gv.fitInView(self.scene.sceneRect())
        
    def postImage(self, msg):
        """ this function is called directly by the ros worker thread,
            don't do any GUI work here
        """
        self.img_msg = msg
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        height, width, channel = self.cv_img.shape
        bytesPerLine = 3 * width
        # must make a deep copy here or experience crash!
        q_img = QImage(self.cv_img.data, width, height, bytesPerLine, QImage.Format_RGB888).copy()
        self.signal.emit(q_img)
        #
        # if this is the monitoring gui, emit the time stamp so
        # it updates at the bottom of the main gui
        #
        if (self.isMonitor):
            self.timeChanged.emit(msg.header.stamp)

    def img_callback(self, msg):
        """ img_callback is called directly by the ros worker thread,
        don't do any GUI work here
        """
        if self.state == "playing":
            self.postImage(msg)
        elif self.state == "replaying":
            self.sync.process(self.topic, msg)
        else:
            rospy.logerr("invalid state: " + self.state)

    def sync_img_callback(self, msg):
        """ sync_img_callback is called directly by the ros worker thread,
        don't do any GUI work here.
        It is called when a complete image is present.
        """
        if self.state == "playing":
            rospy.logerr("got sync callback in playing state!!")
        elif self.state == "replaying":
            self.postImage(msg)
        else:
            rospy.logerr("invalid state!")

    def publish(self):
        """ publish is called when the markup is completed"""
        if not self.img_msg:
            rospy.loginfo('no image received yet')
            return
        rl = RectList()
        rl.header = self.img_msg.header
        mask = np.zeros(self.cv_img.shape[:2], np.uint8)
        for qri in self.gv.rectangles:
            qr = qri.rect()
            x  = int(qr.topLeft().x())
            y  = int(qr.topLeft().y())
            w  = int(qr.width())
            h  = int(qr.height())
            mask[y:y+h, x:x+w] = 255
            r = Rect(x=x, y=y, width=qr.width(), height=qr.height())
            rl.rectangles.append(r)
            rospy.loginfo(self.topic + " rect: [x=%d, y=%d, w=%d h=%d] " \
                          % (x,w,qr.width(), qr.height()))
        self.gv.clearRectangles()
        self.rectPub.publish(rl)
        msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        msg.header = self.img_msg.header
        self.maskPub.publish(msg)

        
class MainWidget(QWidget):
    """ MainWidget is the full GUI, including everything: the views and the buttons"""
    def __init__(self, sync_img_topics, img_topic):
        QWidget.__init__(self, None)
        rospy.loginfo("starting ros worker thread!")
        if not img_topic in sync_img_topics:
            raise Exception(img_topic + ' must be in sync image topics!')
        
        self.time  = None
        self.views = [ViewWidget(self, t, t == img_topic) for t in sync_img_topics]
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

        self.timeField  = QLineEdit(self.buttonsFrame)
        self.timeField.setMaxLength(20)
        self.timeField.setMaximumWidth(200)
        self.playButton = QPushButton('Play', self.buttonsFrame)
        self.stopButton = QPushButton('Stop', self.buttonsFrame)
        self.backButton = QPushButton('Back', self.buttonsFrame)
        self.timeStep   = QSpinBox(self.buttonsFrame)
        self.timeStep.setSingleStep(1)
        self.timeStep.setRange(1,100)
        
        self.forwardButton = QPushButton('Forward', self.buttonsFrame)
        self.duration   = QSpinBox(self.buttonsFrame)
        self.duration.setSingleStep(1)
        self.duration.setRange(1,100)
        self.replayButton = QPushButton('Replay', self.buttonsFrame)
        
        self.publishButton = QPushButton('Publish', self.buttonsFrame)
        self.quitButton = QPushButton('Quit', self.buttonsFrame)
        
        # lay them out from from left to right
        blay.addWidget(self.timeField)
        blay.addWidget(self.playButton)
        blay.addWidget(self.timeStep)
        blay.addWidget(self.stopButton)
        blay.addWidget(self.backButton)
        blay.addWidget(self.timeStep)
        blay.addWidget(self.forwardButton)
        blay.addWidget(self.duration)
        blay.addWidget(self.replayButton)
        blay.addWidget(self.publishButton)
        blay.addWidget(self.quitButton)

        self.playButton.clicked.connect(self.handlePlay)
        self.stopButton.clicked.connect(self.handleStop)
        self.backButton.clicked.connect(self.handleBack)
        self.forwardButton.clicked.connect(self.handleForward)
        self.replayButton.clicked.connect(self.handleReplay)
        self.publishButton.clicked.connect(self.handlePublish)
        self.quitButton.clicked.connect(self.close)

    def sendCmd(self, cmd, v, t):
        """ sendCmd sends a command to the player"""
        try:
            self.rosworker.playerCommand(cmd, v, t)
        except rospy.ServiceException as e:
            rospy.logerr('service call failed: ' + str(e))

    # this decorator was necessary to make it work
    @pyqtSlot(genpy.rostime.Time)
    def handleTimeChanged(self, t):
        self.time = t
        self.timeField.setText(str(t.to_sec()))

    def updateTime(self, delta):
        self.time = self.time + rospy.Duration(delta)
        self.timeField.setText(str(self.time.to_sec()))
    
    def handleBack(self):
        if self.time:
            self.updateTime(-self.timeStep.value())
            self.sendCmd('position', 0, self.time)

    def handleForward(self):
        if self.time:
            self.updateTime(self.timeStep.value())
            self.sendCmd('position', 0, self.time)

    def handleReplay(self):
        if self.time:
            for v in self.views:
                v.setState('replaying')
            self.updateTime(-self.duration.value())
            self.sendCmd('replay', float(self.duration.value()), self.time)
        
    def handlePlay(self, tmp):
        for v in self.views:
            v.setState('playing')
        self.sendCmd('play', 0, rospy.Time(0))

    def handleStop(self, tmp):
        if self.time:
            self.sendCmd('stop', 0, rospy.Time(0))

    def handlePublish(self, tmp):
        for v in self.views:
            v.publish()
    

if __name__ == '__main__':
    rospy.loginfo("starting up")
    app = QApplication(sys.argv)
    w = MainWidget(["/bag_player/image_3",
                    "/bag_player/image_2",
                    "/bag_player/image_6",
                    "/bag_player/image_7",
                    "/bag_player/image_0",
                    "/bag_player/image_1",
                    "/bag_player/image_4",
                    "/bag_player/image_5",],
                   "/bag_player/image_2" # monitoring view
    )
    w.show()
    sys.exit(app.exec_())
