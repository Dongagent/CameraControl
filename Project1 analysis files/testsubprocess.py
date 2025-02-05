import subprocess

# process = subprocess.Popen(["pwsh", '-ExecutionPolicy', 'Unrestricted', "-Command", "ls"], stdout=subprocess.PIPE)

# if process.returncode != 0:
#     print(process.stdout.readlines())
import defaultPose
import copy

def change(au, pos):
    for i in pos:
        au[i - 1] = 255
    return au

df = defaultPose.actionUnitParams['StandardPose']
AU1 = copy.copy(df) # 10, 14
AU2 = copy.copy(df) # 8, 12
AU4 = copy.copy(df) # 11, 15
AU5 = copy.copy(df) # 1,2 （Notice: from 86 to 0）
AU6 = copy.copy(df) # 6+9,7+13
AU7 = copy.copy(df) # 6, 7
AU10 = copy.copy(df) # 30
AU12 = copy.copy(df) # 18,22
LAU12 = copy.copy(df) # 16,17
AU14 = copy.copy(df) # 21,25
AU15 = copy.copy(df) # 19,23
AU16 = copy.copy(df) # 29
AU18 = copy.copy(df) # 26,27
AU20 = copy.copy(df) # 20,24
AU22 = copy.copy(df) # 26,27,29
AU25 = copy.copy(df) # 28,29,32
AU26 = copy.copy(df) # 32
AU43 = copy.copy(df) # 1,2

AU5[0], AU5[1] = 0, 0
# change(AU5

counter = 0
name = ["AU1", "AU2", "AU4", "AU6", "AU7", "AU10", "AU12", "LAU12", "AU14", "AU15", "AU16", "AU18", "AU20", "AU22", "AU25", "AU26", "AU43"]
for i in [[10, 14], [8, 12], [11, 15], [6,9,7,13], [6, 7], [30], [18,22], [16,17], [21,25], [19,23], [29], [26,27], [20,24], [26,27,29], [28,29,32], [32], [1,2]]:
    if counter == 3:
        print("\"AU5\" : {},".format(AU5))

    temp = copy.deepcopy(df)
    print("\"{}\" : {},".format(name[counter], change(temp, i)))
    counter += 1

# print(defaultPose.actionUnitParams['StandardPose'])
# print(AU1)
