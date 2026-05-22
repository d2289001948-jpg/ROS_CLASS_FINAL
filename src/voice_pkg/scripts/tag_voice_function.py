#!/usr/bin/env python3
import rospy
import cv2
import apriltag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

# ==============================
# detect_tag() -> bool
#   订阅相机，等待 2s 稳定，识别 tag36h11 id=1
#   返回 True 表示识别到，False 表示未识别到
# ==============================
def detect_tag():
    if not rospy.core.is_initialized():
        rospy.init_node("tag_voice_node", anonymous=True)

    bridge = CvBridge()
    detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
    current_image = None

    def image_callback(msg):
        nonlocal current_image
        try:
            current_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.sleep(2.0)
    sub.unregister()

    found_tag = False
    if current_image is not None:
        gray = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)
        for tag in tags:
            if tag.tag_id == 1:
                found_tag = True
                break

    cv2.destroyAllWindows()
    if found_tag:
        rospy.loginfo("[DETECT] 识别到 tag1")
    else:
        rospy.loginfo("[DETECT] 未识别到目标")
    return found_tag

# ==============================
# play_result(found)
#   根据识别结果播放对应音频（可在后台线程中调用）
# ==============================
def play_result(found):
    if found:
        os.system("aplay $(rospack find voice_pkg)/sounds/targetfound.wav")
    else:
        os.system("aplay $(rospack find voice_pkg)/sounds/targetnotfound.wav")

# ==============================
# 兼容旧接口（内部直接组合两个函数）
# ==============================
def check_tag1_and_play():
    found = detect_tag()
    play_result(found)

# ==============================
# 测试入口
# ==============================
if __name__ == "__main__":
    print("按回车执行一次识别...")
    input()
    check_tag1_and_play()
