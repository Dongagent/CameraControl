import os
import subprocess

fileName = "out1.mkv"
FRAMERATE = 60
VIDEOSIZE = "1280x720"
DURATION = 5

if os.path.exists(fileName):
    raise

command = "ffmpeg -f v4l2 -framerate " + str(FRAMERATE) + " -video_size " + VIDEOSIZE + " -input_format mjpeg -t " + str(DURATION) + " -i /dev/video2 -t " + str(DURATION) + " -c copy " + fileName
print(command)
# os.
# 

child1 = subprocess.Popen([command], stdout=subprocess.PIPE)
child1.wait()
print(child1.returncode)
# print(child1.stdin)
print(child1.stdout.readlines())
# print(child1.stderr)