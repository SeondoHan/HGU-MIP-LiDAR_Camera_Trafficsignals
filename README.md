

# MIP_Seondo Han

## Object Detection and Depth Estimation of Traffic Signals using mono-camera and low-resolution LiDAR

### Hardware

* LiDAR : Velodyne VLP-16
* Camera : Basler acA800-510uc
* Lens : Kowa LM4NCL

### Software

#### 개발환경

- Windows 10 Education
- CPU = Intel i7-8750H @ 2.20 GHz
- GPU = NVIDIA GeForce GTX 1050 TI
- OpenCV 4.5.1
- CUDA 11.1
- CuDNN 8.1.1

#### transfer_traffic light_labeling_txt.m

ETRI에서 제공받은 traffic light datasets의 labeling을 YOLO에 학습시킬 수 있도록 labeling을 변환해주는 MATLAB 스크립트

traffic light datasets download : https://nanum.etri.re.kr/share/kimjy/etri_traffic_light

- 기존의 Labeling : [Left Top Right Botton Class]
- YOLO의 Labeling : [Class Center_x Center_y Width Hight]

#### videocapture.py

USB로 연결된 webcam을 capture하여 png파일로 저장해주는 파이썬코드

```python
cam = cv2.VideoCapture(1)
```

VideoCapture() 안에 숫자는 webcam number를 의미

랩탑의 경우 내장 webcam의 number = 0,

추가 usb 연결한 webcam의 number = 1.



코드 실행 후, space bar를 누르면 해당 화면 캡처 후 png파일 저장

저장된 파일의 이름은 **opencv_frame_{}.png** 캡처가 될때마다 {}속 숫자가 0부터 증가

esc를 누르면 종료





#### Cali.py

체커보드를 사용하여 카메라의 내부행렬을 구해주는 파이썬코드

nxm 체커보드인지에 따라 입력내용이 다음과 같이 달라짐

해당 코드는 7x9 기준으로 작성되었음

```python
objp = np.zeros((n*m,3), np.float32)
objp[:,:2] = np.mgrid[0:m,0:n].T.reshape(-1,2)
```

```python
ret, corners = cv2.findChessboardCorners(gray, (m,n), None)
```

```python
img = cv2.drawChessboardCorners(img, (m,n), corners2, ret)
```

Calibration을 위해 최소10장 이상의 이미지 사용

```python
images = glob.glob('*.png')
```

해당 폴더 내 모든 png파일을 import한다는 내용

```python
cv2.waitKey(10000)
```

for 문이 10초의 한번씩 돌아감. 안의 숫자를 필요에 맞게 수정가능. 단위[ms]

출력창에서 카메라 내부행렬 확인 가능





#### darknet

darknet github : https://github.com/AlexeyAB/darknet/

Bounding box 좌표

darknet.sln 실행

image_opencv.cpp 실행

```c++
extern "C" void draw_detections_cv_v3(mat_cv* mat, detection *dets, int num, float thresh, char **names, image **alphabet, int classes, int ext_output)
```

를 통해 bounding box 좌표에 접근 가능

```c
double distance_seondo;
if (class_id == 0) {				// object의 class_id. class에 따라 크기가 달라질 경우, if문을 다르게 설정해야함

    // camera frame [m]
    double c_seondo = 4.785156;		// pixel -> m 로 바꾸는 변환 상수
    double height_seondo = b.h * show_img->rows;
    distance_seondo = 1.71 * 3.5 * 1000 / (height_seondo * c_seondo); // 1.71 대신 object의 크기 입력, 단위[m]

    printf("%.2f", distance_seondo);


    // Lidar frame xyz
    double center_seondo_x = (left + right) * 0.5;
    double center_seondo_y = (top + bot) * 0.5;
    double cz_seondo = (distance_seondo + 2.753252) / (-0.000248 * center_seondo_x - 0.000027 * center_seondo_y + 1.185288);
    double cx_seondo = center_seondo_x * cz_seondo;
    double cy_seondo = center_seondo_y * cz_seondo;
    double lidar_seondo_xm = 0.000645 * cx_seondo - 0.000010 * cy_seondo - 0.254882 * cz_seondo + 0.027709;
    double lidar_seondo_ym = distance_seondo;
    double lidar_seondo_zm = 0.000001 * cx_seondo - 0.000689 * cy_seondo + 0.329190 * cz_seondo - 0.776078;

    //printf("\nlidar_x: %.2f[m]\tlidar_y: %.2f[m]\tlidar_z: %.2f[m]\n", lidar_seondo_xm, lidar_seondo_ym, lidar_seondo_zm);

    // Lidar frame r-theta
    double lidar_seondo_r = sqrt(pow(lidar_seondo_xm, 2) + pow(lidar_seondo_ym, 2) + pow(lidar_seondo_zm, 2));
    double lidar_seondo_azimuth = atan2(lidar_seondo_xm, lidar_seondo_ym);
    double lidar_seondo_theta = asin(lidar_seondo_zm / lidar_seondo_r);

    //printf("\nlidar_r: %.2f[m]\tlidar_azimuth: %.2f[rad]\tlidar_vertical: %.2f[rad]\n", lidar_seondo_r, lidar_seondo_azimuth, lidar_seondo_theta);

    // Lidar frame r-theta degree-centimeter
    double lidar_seondo_rcm = lidar_seondo_r * 100;
    double lidar_seondo_azimuthdeg = lidar_seondo_azimuth * 180 / PI;
    double lidar_seondo_verticaldeg = lidar_seondo_theta * 180 / PI;

    if (lidar_seondo_azimuthdeg < 0) {
        lidar_seondo_azimuthdeg += 360;
    }

    //printf("\nlidar_r: %.2f[cm]\tlidar_azimuth: %.2f[deg]\tlidar_vertical: %.2f[deg]\n", lidar_seondo_rcm, lidar_seondo_azimuthdeg, lidar_seondo_verticaldeg);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Left - Top == LT
    // Lidar frame xyz
    double cz_seondo_LT = (distance_seondo + 2.753252) / (-0.000248 * left - 0.000027 * top + 1.185288);
    double cx_seondo_LT = left * cz_seondo_LT;
    double cy_seondo_LT = top * cz_seondo_LT;
    double lidar_seondo_xm_LT = 0.000645 * cx_seondo_LT - 0.000010 * cy_seondo_LT - 0.254882 * cz_seondo_LT + 0.027709;
    double lidar_seondo_ym_LT = distance_seondo;
    double lidar_seondo_zm_LT = 0.000001 * cx_seondo_LT - 0.000689 * cy_seondo_LT + 0.329190 * cz_seondo_LT - 0.776078;

    // Lidar frame r-theta
    double lidar_seondo_r_LT = sqrt(pow(lidar_seondo_xm_LT, 2) + pow(lidar_seondo_ym_LT, 2) + pow(lidar_seondo_zm_LT, 2));
    double lidar_seondo_azimuth_LT = atan2(lidar_seondo_xm_LT, lidar_seondo_ym_LT);
    double lidar_seondo_theta_LT = asin(lidar_seondo_zm_LT / lidar_seondo_r_LT);

    //printf("\nlidar_r: %.2f[m]\tlidar_azimuth: %.2f[rad]\tlidar_vertical: %.2f[rad]\n", lidar_seondo_r, lidar_seondo_azimuth, lidar_seondo_theta);

    // Lidar frame r-theta degree-centimeter
    double lidar_seondo_rcm_LT = lidar_seondo_r_LT * 100;
    double lidar_seondo_azimuthdeg_LT = lidar_seondo_azimuth_LT * 180 / PI;
    double lidar_seondo_verticaldeg_LT = lidar_seondo_theta_LT * 180 / PI;

    if (lidar_seondo_azimuthdeg_LT < 0) {
        lidar_seondo_azimuthdeg_LT += 360;
    }

    //printf("LT");
    //printf("\nlidar_r: %.2f[cm]\tlidar_azimuth: %.2f[deg]\tlidar_vertical: %.2f[deg]\n", lidar_seondo_rcm_LT, lidar_seondo_azimuthdeg_LT, lidar_seondo_verticaldeg_LT);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Right - Bottom == RB
    // Lidar frame xyz

    double cz_seondo_RB = (distance_seondo + 2.753252) / (-0.000248 * right - 0.000027 * bot + 1.185288);
    double cx_seondo_RB = right * cz_seondo;
    double cy_seondo_RB = bot * cz_seondo;
    double lidar_seondo_xm_RB = 0.000645 * cx_seondo_RB - 0.000010 * cy_seondo_RB - 0.254882 * cz_seondo_RB + 0.027709;
    double lidar_seondo_ym_RB = distance_seondo;
    double lidar_seondo_zm_RB = 0.000001 * cx_seondo_RB - 0.000689 * cy_seondo_RB + 0.329190 * cz_seondo_RB - 0.776078;

    //printf("\nlidar_x: %.2f[m]\tlidar_y: %.2f[m]\tlidar_z: %.2f[m]\n", lidar_seondo_xm, lidar_seondo_ym, lidar_seondo_zm);

    // Lidar frame r-theta
    double lidar_seondo_r_RB = sqrt(pow(lidar_seondo_xm_RB, 2) + pow(lidar_seondo_ym_RB, 2) + pow(lidar_seondo_zm_RB, 2));
    double lidar_seondo_azimuth_RB = atan2(lidar_seondo_xm_RB, lidar_seondo_ym_RB);
    double lidar_seondo_theta_RB = asin(lidar_seondo_zm_RB / lidar_seondo_r_RB);

    //printf("\nlidar_r: %.2f[m]\tlidar_azimuth: %.2f[rad]\tlidar_vertical: %.2f[rad]\n", lidar_seondo_r, lidar_seondo_azimuth, lidar_seondo_theta);

    // Lidar frame r-theta degree-centimeter
    double lidar_seondo_rcm_RB = lidar_seondo_r_RB * 100;
    double lidar_seondo_azimuthdeg_RB = lidar_seondo_azimuth_RB * 180 / PI;
    double lidar_seondo_verticaldeg_RB = lidar_seondo_theta_RB * 180 / PI;

    if (lidar_seondo_azimuthdeg_RB < 0) {
        lidar_seondo_azimuthdeg_RB += 360;
    }

    //printf("RB");
    //printf("\nlidar_r: %.2f[cm]\tlidar_azimuth: %.2f[deg]\tlidar_vertical: %.2f[deg]\n", lidar_seondo_rcm_RB, lidar_seondo_azimuthdeg_RB, lidar_seondo_verticaldeg_RB);
}
```

해당 코드를

```c++
cv::rectangle(*show_img, pt1, pt2, color, width, 8, 0);
```

위에 삽임

해당코드는 bounding box의 왼쪽상단(LT), 정중앙(center), 우측하단(RB)의 좌표를 라이다 좌표계의 azimuth/vertical/distance 로 변환해주는 코드이다.

곱해지는 상수는 LiDAR_Camera_Calibration의 결과로 나온 행렬 

![matrix1](C:\Users\hansd0118\Documents\GitHub\HGU-MIP-LiDAR_Camera_Trafficsignals\image\matrix1.PNG)

의 역행렬인

![matrix2](C:\Users\hansd0118\Documents\GitHub\HGU-MIP-LiDAR_Camera_Trafficsignals\image\matrix2.PNG)

를 곱하여 준다.

예를 들어,

![matrix3](C:\Users\hansd0118\Documents\GitHub\HGU-MIP-LiDAR_Camera_Trafficsignals\image\matrix3.PNG)

라고 할때

```c
double cz_seondo = (distance_seondo + h) / (e * center_seondo_x + f * center_seondo_y + g);
double cx_seondo = center_seondo_x * cz_seondo;
double cy_seondo = center_seondo_y * cz_seondo;
double lidar_seondo_xm = a * cx_seondo + b * cy_seondo + c * cz_seondo + d;
double lidar_seondo_ym = distance_seondo;
double lidar_seondo_zm = i * cx_seondo + j * cy_seondo + k * cz_seondo + l;
```

이다.







YOLO실행방법

1. darknet/x64 폴더로 이동 후, 경로창에서 cmd 실행

   ![darknet_cmd](C:\Users\hansd0118\Documents\GitHub\HGU-MIP-LiDAR_Camera_Trafficsignals\image\darknet_cmd.PNG)

   

2. image를 사용하여 object detection할 때, cmd 창에

   **darknet.exe detector test {@.data} {@.cfg} {@.weights} {iamge}**

   입력

   {}속 파일 경로는 x64 폴더 기준. 만약 yolov4.cfg 파일이 x64폴더에 있다면 {@.cfg} 자리에 yolov4.cfg 입력.

   만약 yolov4.cfg 파일이 x64/config폴더에 있다면 {@.cfg} 자리에 config/yolov4.cfg 입력.

3. video를 사용하여 object detection할 때, cmd 창에

   **darknet.exe detector demo {@.data} {@.cfg} {@.weights} {video}**

   입력

4. webcam을 이용하여 object detection할 때, cmd 창에

   **darknet.exe detector demo {@.data} {@.cfg} {@.weights} -c {webcam number}**

   입력

   랩탑의 경우 내장 webcam의 number = 0,

   추가 usb 연결한 webcam의 number = 1.

   

#### VelodyneCapture

VelodyneCapture github : https://github.com/UnaNancyOwen/VelodyneCapture

사용을 위해 boost.Asio 개발환경 구축 필요

https://ndb796.tistory.com/113

Azimuth[degree], Vertical[degree], Distance[centimeter] 로 출력



#### LiDAR_Camera_Calibration_with_Boxes_Final

폴더 내 README.docx 참고

*해당 폴더내 모든 내용은 한동대학교 2019년 졸업 연구로 김예함, 이예은에 의해 제작되었음.





