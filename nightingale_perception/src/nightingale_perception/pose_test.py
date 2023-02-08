import cv2 
import mediapipe as mp
import numpy as np
import time
import math

class Landmarks:
    # assign the integers to the useful ones
    NOSE = 0
    LEFT_EYE = 2
    RIGHT_EYE = 5
    LEFT_EAR = 7
    RIGHT_EAR = 8
    MOUTH_LEFT = 9
    MOUTH_RIGHT = 10
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_ELBOW = 13
    RIGHT_ELBOW = 14
    LEFT_WRIST = 15
    RIGHT_WRIST = 16
    LEFT_THUMB = 21
    RIGHT_THUMB = 22
    LEFT_HIP = 23
    RIGHT_HIP = 24

def euclid_dist_3d(lm_1, lm_2):
    x_dist = lm_1.x - lm_2.x
    y_dist = lm_1.y - lm_2.y
    z_dist = lm_1.z - lm_2.z

    distance = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2) 
    return distance

def euclid_dist_2d(lm_1, lm_2):
    x_dist = lm_1.x - lm_2.x
    y_dist = lm_1.y - lm_2.y

    distance = math.sqrt(x_dist**2 + y_dist**2) 
    return distance

def calc_arm_len(thumb_lm, elbow_lm, shoulder_lm):
    arm_length = 0
    bicep_length = euclid_dist_3d(shoulder_lm, elbow_lm)
    forearm_length = euclid_dist_3d(elbow_lm, thumb_lm)
    return bicep_length + forearm_length

def main():
    lm = Landmarks()
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles

    prev_frame_time = 0
    new_frame_time = 0 

    # For webcam input:
    cap = cv2.VideoCapture(0)
    with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
      while cap.isOpened():
        success, image = cap.read()

        # fps
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = str(int(fps))
        #print("fps",fps)

        if not success:
          print("Ignoring empty camera frame.")
          # If loading a video, use 'break' instead of 'continue'.
          continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        #landmarks_list = results.pose_world_landmarks.landmark
        landmarks_list = results.pose_landmarks.landmark

        # hand pose detection
        visible_thresh = 0.7
        landmark_poses = []
        hand_pose_confirmed = 5
        hand_pose_counter = 0
        total_time = 0
        allowed_time = 45 # seconds
        while hand_pose_counter < hand_pose_confirmed and total_time < allowed_time:
            # probably start up the media pose pipeline in here

            for landmark in hand_lm:
                if landmark.visibility > visible_thresh:
                    if abs(euclid_dist_2d(lm_new, lm_old)) not in range(hand_delta_thresh):
                        landmark_poses[lm_enum] = lm_new
                        hand_pose_counter = -0.5 
            hand_pose_counter = hand_pose_counter + 0.5
            time.sleep(0.5)
        # should have landmarks to use






        if landmarks_list[lm.RIGHT_THUMB] and landmarks_list[lm.RIGHT_ELBOW] and landmarks_list[lm.RIGHT_SHOULDER]:
            right_thumb_lm = landmarks_list[lm.RIGHT_THUMB]
            right_elbow_lm = landmarks_list[lm.RIGHT_ELBOW]
            right_shoulder_lm = landmarks_list[lm.RIGHT_SHOULDER]
            right_arm_length = calc_arm_len(right_thumb_lm, right_elbow_lm, right_shoulder_lm)
            print("Right arm length", right_arm_length)
        #print(landmarks_list[lm.RIGHT_EYE])

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
        if cv2.waitKey(5) & 0xFF == 27:
          break
    cap.release()


def point_overlay(point, depth_img):
    # depth img is ros image message
    data = depth_img.data
    x_pos = int(point.x * data.width)
    y_pos = int(point.y * data.height)
    z_pos = y_pos*data.step + x_pos
    # possible some conversion factor to meters
    return z_pos
    


if __name__ == "__main__":
    main()


