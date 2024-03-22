For errors that look like this:
Connected cameras: [{socket: CAM_A, sensorName: IMX214, width: 4208, height: 3120, orientation: AUTO, supportedTypes: [COLOR], hasAutofocus: 0, hasAutofocusIC: 1, name: 
color}, {socket: CAM_B, sensorName: OV7251, width: 640, height: 480, orientation: AUTO, supportedTypes: [MONO], hasAutofocus: 0, hasAutofocusIC: 0, name: left}, {socket: 
CAM_C, sensorName: OV7251, width: 640, height: 480, orientation: AUTO, supportedTypes: [MONO], hasAutofocus: 0, hasAutofocusIC: 0, name: right}]
Usb speed: HIGH
Device name: OAK-D-LITE  Product name: OAK-D-LITE
Unable to init server: Could not connect: Connection refused
Traceback (most recent call last):
  File "rgb_preview.py", line 42, in <module>
    cv2.imshow("rgb", inRgb.getCvFrame())
cv2.error: OpenCV(4.9.0) /tmp/build_opencv/opencv/modules/highgui/src/window_gtk.cpp:638: error: (-2:Unspecified error) Can't initialize GTK backend in function 
'cvInitSystem'

The fix is to uncomment the line
CAMERA_INDEX = 0
in the myconfig.py file

Thanks raymond for showing me this fix!
