pro_distortions = [
    -0.4662440617945527,
    -0.1306917894003836,
    -0.00747551962097448,
    -0.00891657451203357,
    -1.262628839942179
]
pro_undistortions = [
    0.3319747182723289,
    -0.03158516887583922,
    0.008540143299613964,
    0.0038913658441293054,
    1.1623888957076565
]

dist = DistortionCV(resolution=(2408, 1505))
dist.setDistortion(pro_distortions)
dist.setUndistortion(pro_undistortions)
imgs_x = generate_fringe_bmps(pro_distortion=dist, patterns=patterns, direction='x')
imgs_y = generate_fringe_bmps(pro_distortion=dist, patterns=patterns, direction='y')