import cv2

# Webカメラ
DEVICE_ID = 1 

WIDTH = 1280
HEIGHT = 720
FPS = 60

cap = cv2.VideoCapture (DEVICE_ID)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

def decode_fourcc(v):
    v = int(v)
    return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

# フォーマット・解像度・FPSの取得
fourcc = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)
print("fourcc:{} fps:{}　width:{}　height:{}".format(fourcc, fps, width, height))