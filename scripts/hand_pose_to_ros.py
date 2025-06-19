import cv2
import mediapipe as mp
import roslibpy
import json
import math

mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
pose = mp_pose.Pose()
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
ros = roslibpy.Ros(host='localhost', port=9090)  # 若 rosbridge 不在本機，請改 host
ros.run()
topic = roslibpy.Topic(ros, '/hand_to_arm', 'std_msgs/String')

cap = cv2.VideoCapture(0)
try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(image)
        hand_results = hands.process(image)
        joint_points = {}

        # 1. 腰部（右臀 landmark 24）
        if pose_results.pose_landmarks:
            waist = pose_results.pose_landmarks.landmark[24]
            joint_points['joint_1'] = [waist.x, waist.y, waist.z]
            # 2. 肩膀（右肩）
            shoulder = pose_results.pose_landmarks.landmark[12]
            joint_points['joint_2'] = [shoulder.x, shoulder.y, shoulder.z]
            # 3. 手軸（右手肘）
            elbow = pose_results.pose_landmarks.landmark[14]
            joint_points['joint_3'] = [elbow.x, elbow.y, elbow.z]
            # 4. 手腕（右手腕）
            wrist = pose_results.pose_landmarks.landmark[16]
            joint_points['joint_4'] = [wrist.x, wrist.y, wrist.z]

        # 5. 手指關節（右手中指第一關節）與 6. 指尖（右手中指指尖）
        if hand_results.multi_hand_landmarks:
            finger_joint = hand_results.multi_hand_landmarks[0].landmark[10]
            joint_points['joint_5'] = [finger_joint.x, finger_joint.y, finger_joint.z]
            fingertip = hand_results.multi_hand_landmarks[0].landmark[12]
            joint_points['joint_6'] = [fingertip.x, fingertip.y, fingertip.z]

        # 只要六點都偵測到才發送
        if len(joint_points) == 6:
            topic.publish(roslibpy.Message({'data': json.dumps(joint_points)}))

        # --- 只顯示六個指定骨架點與連線 ---
        joint_coords = []
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        for i, name in enumerate(joint_names):
            if name in joint_points:
                x, y, z = joint_points[name]
                h, w, _ = frame.shape
                cx, cy = int(x * w), int(y * h)
                joint_coords.append((cx, cy))
                # 畫點
                cv2.circle(frame, (cx, cy), 8, (0, 255, 255), -1)
                cv2.putText(frame, name, (cx+5, cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
        # 畫連線
        for i in range(len(joint_coords)-1):
            cv2.line(frame, joint_coords[i], joint_coords[i+1], (255, 0, 0), 3)

        cv2.imshow('Hand & Pose Detection', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    topic.unadvertise()
    ros.terminate()

def hand_to_joint_angles(hand_joints):
    # 每個關節的角度範圍（degree）
    joint_angle_ranges = [
        (0, 90),   # joint_1
        (-60, 30),  # joint_2
        (0, 90),   # joint_3
        (-90, 90), # joint_4
        (-60, 60),   # joint_5
        (-90, 90),   # joint_6
    ]
    angles = []
    for i in range(6):
        x = hand_joints[f'joint_{i+1}'][0]
        min_deg, max_deg = joint_angle_ranges[i]
        # 將 x (0~1) 映射到角度範圍
        angle_deg = min_deg + (max_deg - min_deg) * x
        # 轉成弧度
        angle_rad = math.radians(angle_deg)
        angles.append(angle_rad)
    return angles
