import cv2
import numpy as np

def generatePattern(CheckerboardSize, Nx_cor, Ny_cor):
    '''
    自定义生成棋盘
    :param CheckerboardSize: 棋盘格大小,此处100即可
    :param Nx_cor: 棋盘格横向内角数
    :param Ny_cor: 棋盘格纵向内角数
    :return:
    '''
    black = np.zeros((CheckerboardSize, CheckerboardSize, 3), np.uint8)
    white = np.zeros((CheckerboardSize, CheckerboardSize, 3), np.uint8)
    black[:] = [0, 0, 0]  # 纯黑色
    white[:] = [255, 255, 255]  # 纯白色

    black_white = np.concatenate([black, white], axis=1)
    black_white2 = black_white
    white_black = np.concatenate([white, black], axis=1)
    white_black2 = white_black

    # 横向连接
    if Nx_cor % 2 == 1:
        for i in range(1, (Nx_cor+1) // 2):
            black_white2 = np.concatenate([black_white2, black_white], axis=1)
            white_black2 = np.concatenate([white_black2, white_black], axis=1)
    else:
        for i in range(1, Nx_cor // 2):
            black_white2 = np.concatenate([black_white2, black_white], axis=1)
            white_black2 = np.concatenate([white_black2, white_black], axis=1)
        black_white2 = np.concatenate([black_white2, black], axis=1)
        white_black2 = np.concatenate([white_black2, white], axis=1)

    jj = 0
    black_white3 = black_white2
    for i in range(0, Ny_cor):
        jj += 1
        # 纵向连接
        if jj % 2 == 1:
            black_white3 = np.concatenate((black_white3, white_black2))  # =np.vstack((img1, img2))
        else:
            black_white3 = np.concatenate((black_white3, black_white2))  # =np.vstack((img1, img2))

    cv2.imshow('', black_white3)
    cv2.imwrite('pattern.jpg', black_white3)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    generatePattern(100, 9, 6)