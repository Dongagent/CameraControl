# prototype anger related axis: [[1, 2], [6, 7], [11, 15]]]
# prototype happiness related axis: [[6, 7], [9, 13], [16, 17], [18, 22]]]

headYaw_fix = 105
prototypeFacialExpressions = {
    'anger': [
        [0, 0, 128, 128, 128, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, headYaw_fix],
        [10, 10, 128, 128, 128, 245, 245, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, headYaw_fix],
        # [10, 10, 128, 128, 128, 255, 255, 0, 0, 0, 245, 0, 0, 0, 245, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, headYaw_fix],
        [0, 0, 128, 128, 128, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 0, 0, 255, 128, 128, headYaw_fix],
        [0, 0, 128, 128, 128, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 235, 235, 0, 0, 255, 128, 128, headYaw_fix],
        [0, 0, 128, 128, 128, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 215, 215, 0, 0, 255, 128, 128, headYaw_fix],
    ], # first 2 are closing mouth, last 3 are opening mouth
    "happiness" : [
        [86, 86, 128, 128, 128, 255, 255, 0, 255, 0, 0, 0, 255, 0, 0, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, headYaw_fix],
        [86, 86, 128, 128, 128, 245, 245, 0, 255, 0, 0, 0, 255, 0, 0, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, headYaw_fix],
        # [86, 86, 128, 128, 128, 255, 255, 0, 245, 0, 0, 0, 245, 0, 0, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, headYaw_fix],
        [86, 86, 128, 128, 128, 255, 255, 0, 255, 0, 0, 0, 255, 0, 0, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 255, 255, 0, 0, 255, 128, 128, headYaw_fix],
        [86, 86, 128, 128, 128, 255, 255, 0, 255, 0, 0, 0, 255, 0, 0, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 235, 235, 0, 0, 255, 128, 128, headYaw_fix],
        [86, 86, 128, 128, 128, 255, 255, 0, 255, 0, 0, 0, 255, 0, 0, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 215, 215, 0, 0, 255, 128, 128, headYaw_fix],
    ], # first 2 are closing mouth, last 3 are opening mouth
}

BOFacialExpressions = {
    "anger": [
        [0, 0, 128, 128, 128, 140, 140, 163, 0, 0, 255, 163, 0, 0, 255, 255, 255, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 255, 225, 255, 0, 255, 128, 128, headYaw_fix],
        [3, 3, 128, 128, 128, 140, 140, 240, 0, 0, 243, 240, 0, 0, 243, 204, 204, 0, 172, 107, 0, 0, 172, 107, 0, 0, 0, 91, 168, 255, 0, 243, 128, 128, headYaw_fix],
        [0, 0, 128, 128, 128, 140, 140, 205, 0, 0, 255, 205, 0, 0, 255, 213, 213, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 218, 152, 255, 0, 255, 128, 128, headYaw_fix],
        [39, 39, 128, 128, 128, 140, 140, 216, 0, 0, 255, 216, 0, 0, 255, 203, 203, 0, 32, 51, 0, 0, 32, 51, 0, 0, 0, 4, 231, 255, 0, 255, 128, 128, headYaw_fix],
        [20, 20, 128, 128, 128, 140, 140, 236, 0, 0, 247, 236, 0, 0, 247, 203, 203, 0, 32, 51, 0, 0, 132, 151, 0, 0, 0, 54, 81, 255, 0, 255, 128, 128, headYaw_fix],
        # [0, 0, 128, 128, 128, 140, 140, 164, 0, 0, 255, 164, 0, 0, 255, 148, 148, 0, 235, 130, 0, 0, 235, 130, 0, 0, 0, 135, 123, 255, 0, 255, 128, 128, headYaw_fix],
    ],
    "happiness": [
        [178, 178, 128, 128, 128, 140, 140, 0, 159, 20, 0, 0, 159, 20, 0, 171, 171, 255, 0, 217, 0, 255, 0, 217, 0, 0, 0, 255, 0, 95, 0, 76, 128, 128, headYaw_fix],
        [121, 121, 128, 128, 128, 64, 64, 0, 95, 172, 0, 0, 95, 172, 0, 196, 196, 0, 53, 160, 0, 0, 53, 160, 0, 0, 0, 156, 39, 111, 0, 39, 128, 128, headYaw_fix],
        [127, 127, 128, 128, 128, 95, 95, 0, 4, 198, 45, 0, 4, 198, 45, 198, 198, 168, 0, 37, 0, 168, 0, 37, 0, 0, 0, 230, 84, 11, 0, 154, 128, 128, headYaw_fix],
        [93, 93, 128, 128, 128, 78, 78, 50, 0, 98, 16, 50, 0, 98, 16, 211, 211, 223, 0, 129, 0, 223, 0, 129, 0, 0, 0, 144, 2, 148, 0, 48, 128, 128, headYaw_fix],
        [33, 33, 128, 128, 128, 78, 78, 50, 0, 98, 16, 50, 0, 98, 16, 211, 211, 223, 0, 129, 0, 223, 0, 129, 0, 0, 0, 144, 2, 148, 0, 48, 128, 128, headYaw_fix],
        # [33, 33, 128, 128, 128, 78, 78, 50, 0, 98, 16, 50, 0, 98, 16, 211, 211, 223, 0, 129, 0, 223, 0, 129, 0, 0, 0, 144, 2, 148, 0, 148, 128, 128, headYaw_fix],
    ]
}