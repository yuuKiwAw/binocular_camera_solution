import cv2
import os
import time

CAM_INDEX = 1
FRAME_SIZE = (2560, 720)

show_size = (1280, 480)

save_dir= ""

def init_dir():
    global save_dir
    current_path = os.path.abspath(__file__)
    current_path = os.path.abspath(os.path.dirname(current_path) + os.path.sep + ".")

    if not os.path.exists(current_path + "\\save_img"):
        os.mkdir(current_path + "\\save_img")

    save_dir = current_path + "\\save_img\\" + time.strftime("%Y%m%d-%H%M%S")
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

def print_info():
    print("====================== INFO ======================")
    print("VIDEOCAPTURE INDEX:      {0}".format(CAM_INDEX))
    print("DUAL CAMERA IMAGE SIZE:  [W: {0}, H: {1}]".format(FRAME_SIZE[0], FRAME_SIZE[1]))
    print("DISPLAY IMAGE SIZE:      [W: {0}, H: {1}]".format(show_size[0], show_size[1]))
    print("IMAGE SAVE PATH:         {0}".format(save_dir))
    print("")
    print("PRESS 'S' SAVE IMAGE")
    print("PRESS 'ESC' QUIT")
    print("====================== INFO ======================")

def main():
    print("============== DUAL CAMERA SHUTTER ==============")
    print("LOADING DUAL CAMERA PLEASE WAITING...")

    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])

    init_dir()
    print_info()

    save_count = 0 #图片保存计数

    while True:
        ret, frame= cap.read()
        if not ret:
            break

        resize_frame = cv2.resize(frame, show_size)
        cv2.imshow("dual_camera", resize_frame)
        key = cv2.waitKey(1)
        if key == 27:
            break

        if key == ord('s'):
            left_img_path = "{0}\\left_{1}.jpg".format(save_dir, save_count)
            right_img_path = "{0}\\right_{1}.jpg".format(save_dir, save_count)

            # [Ymin:Ymax,Xmin:Xmax]
            left_img = frame[0:FRAME_SIZE[1], 0:int(FRAME_SIZE[0]/2)]
            right_img = frame[0:FRAME_SIZE[1], int(FRAME_SIZE[0]/2):FRAME_SIZE[0]]
            cv2.imwrite(left_img_path, left_img)
            cv2.imwrite(right_img_path, right_img)
            save_count += 1
            print("[INFO]left  img save: {0}".format(left_img_path))
            print("[INFO]right img save: {0}".format(right_img_path))
            print("")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()