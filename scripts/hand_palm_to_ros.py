import cv2
import mediapipe as mp
import roslibpy
import json

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
ros = roslibpy.Ros(host='localhost', port=9090)
ros.run()
topic = roslibpy.Topic(ros, '/palm_target', 'std_msgs/String')

cap = cv2.VideoCapture(0)
try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(image)
        palm_pos = None
        if results.multi_hand_landmarks:
            # 0號點為手腕，9號點為掌心附近
            palm = results.multi_hand_landmarks[0].landmark[9]
            h, w, _ = frame.shape
            cx, cy = int(palm.x * w), int(palm.y * h)
            palm_pos = {'x': palm.x, 'y': palm.y}
            # 畫出手心
            cv2.circle(frame, (cx, cy), 12, (0, 255, 255), -1)
            cv2.putText(frame, "Palm", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            # 發送到 ROS
            topic.publish(roslibpy.Message({'data': json.dumps(palm_pos)}))
        cv2.imshow('Palm Detection', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    topic.unadvertise()
    ros.terminate()
