#!/usr/bin/env python

import math
import PySimpleGUI as sg
import rospy
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3, Pose
import uuid

# https://qiita.com/kitasenjudesign/items/e785e00736161ec238ae
def chokkou(r,theta,phi):
  x = r * math.sin(theta) * math.cos(phi)
  y = r * math.sin(theta) * math.sin(phi)
  z = r * math.cos(theta)
  return (x,y,z)
# 条件に合致する最初の要素のインデックスを見つける
def find_first_index(lst, key, value):
    for index, item in enumerate(lst):
        if item[key] == value:
            return index
    return None 

def build():
    contents = [
        [
            sg.Column([[sg.Text()], [sg.Text(text="Position")], [sg.Text(text="Rotation")]]),
            sg.Column([[sg.Text(text="X")], [sg.InputText(key="x")], [sg.InputText(key="rx")]]),
            sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y")], [sg.InputText(key="ry")]]),
            sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z")], [sg.InputText(key="rz")]])
        ], 
        [
            sg.Tree(key="-tree-", data=sg.TreeData(), enable_events=True, show_expanded=True)
        ], 
        [
            sg.Button(button_text="Add", key="-add-"), sg.Button(button_text="Edit", key="-edit-"), sg.Button(button_text="Remove", key="-remove-")
        ]
    ]
    
    return contents

def buildTree(cams: "list[Cam]"):
    treeData = sg.TreeData()
    for cam in cams:
        treeData.insert(cam.parentKey, cam.key, cam.label, [])
    return treeData

def buildAdd():
    contents = [
        [
            sg.Column([[sg.Text()], [sg.Text(text="Position")], [sg.Text(text="Rotation")]]),
            sg.Column([[sg.Text(text="X")], [sg.InputText(key="x")], [sg.InputText(key="rx")]]),
            sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y")], [sg.InputText(key="ry")]]),
            sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z")], [sg.InputText(key="rz")]])
        ], 
        [
            sg.Text(text="ラベル"), sg.InputText(key="_label_")  
        ],
        [
            sg.Button(button_text="Confirm", key="-add-confirm-"), sg.Button(button_text="Cancel", key="-add-cancel-")
        ]
    ]
    return contents

def buildEdit():
    contents = [
        [
            sg.Column([[sg.Text()], [sg.Text(text="Position")], [sg.Text(text="Rotation")]]),
            sg.Column([[sg.Text(text="X")], [sg.InputText(key="x")], [sg.InputText(key="rx")]]),
            sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y")], [sg.InputText(key="ry")]]),
            sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z")], [sg.InputText(key="rz")]])
        ], 
        [
            sg.Text(text="ラベル"), sg.InputText(key="_label_")  
        ],
        [
            sg.Button(button_text="Confirm", key="-edit-confirm-"), sg.Button(button_text="Cancel", key="-edit-cancel-")
        ]
    ]
    return contents

def buildRemove():
    contents = [
        [
            sg.Text(text="この視点を削除します。")
        ],
        [
            sg.Button(button_text="Confirm", key="-remove-confirm-"), sg.Button(button_text="Cancel", key="-remove-cancel-")
        ]
    ]
    return contents
    

class Cam(dict):
    def __init__(self, parentKey, key, label, x, y, z, rx, ry, rz):
        super().__init__()
        self.__dict__ = self
        self.parentKey = parentKey
        self.key = key
        self.label = label
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
    
    def toCameraPlacement(self):
        cp = CameraPlacement()
        p = Point(self.x, self.y, self.z)
        cp.eye.point= p
        cp.eye.header.frame_id = "base_link"
        f = Vector3(0, 0, 1)
        cp.focus.point = f
        cp.focus.header.frame_id = "base_link"
        return cp
    
    def print(self):
        print(self.label, self.x, self.y, self.z, self.rx, self.ry, self.rz)

class CameraManager:
    def __init__(self):
        self.cams: list[Cam] = []
        
        rospy.init_node("camera-manager", anonymous = True)
        self.pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)
        self.sub = rospy.Subscriber("/rviz/current_camera_pose", Pose, self.updateCurrentCam)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0
    
    def getCam(self, camKey):
        cam = next(filter(lambda cam: cam.key == camKey, self.cams))
        return cam
    
    def addCam(self, cam: Cam):
        self.cams.append(cam)
        return cam
    
    def updateCam(self, cam: Cam):        
        index = find_first_index(self.cams, "key", cam.key)
        self.cams[index] = cam
        return self.cams[index]
    
    def removeCam(self, camKey):
        index = find_first_index(self.cams, "key", camKey)
        self.cams.pop(index)    
        return
    
    def selectCam(self, camKey):
        cam = next(filter(lambda cam: cam.key == camKey, self.cams))
        self.pub.publish(cam.toCameraPlacement())
        return cam
        
    def loadCams(self, cams: "list[Cam]"):
        self.cams = cams
        return self.cams
    
    def loadCamsFromLocal(self):
        cam_list = rospy.get_param("/cam_list")
        cams = [Cam(_["parentKey"], _["key"], _["label"], _["x"], _["y"], _["z"], _["rx"], _["ry"], _["rz"]) for _ in cam_list]
        self.cams = cams
        return self.cams
    
    def updateCurrentCam(self, cp: Pose):
        self.x = cp.position.x
        self.y = cp.position.y
        self.z = cp.position.z
        self.rx = cp.orientation.x
        self.ry = cp.orientation.y
        self.rz = cp.orientation.z

if __name__ == "__main__":
    app = CameraManager()
    
    contents = build()
    window = sg.Window("Rviz Camera Manager", contents, finalize=True)
    
    cams = app.loadCamsFromLocal()
    treeData = buildTree(cams)
    window["-tree-"].update(treeData)

    window["-tree-"].bind('<Double-1>', "double-click-")
    
    popup = None
    keyForPopup = ""
    
    while True:
        event, values = window.read(timeout=100, timeout_key='-timeout-')    
        popupEvent, popupValues = popup.read(timeout=100, timeout_key='-timeout-popup-') if not popup == None else (None, None)

        window["x"].update(str(app.x))
        window["y"].update(str(app.y))
        window["z"].update(str(app.z))
        window["rx"].update(str(app.rx))
        window["ry"].update(str(app.ry))
        window["rz"].update(str(app.rz))

        if event == sg.WIN_CLOSED:
            print('exit')
            break
        
        # ツリー操作
        if event == '-tree-double-click-':
            key = values["-tree-"][0]
            app.selectCam(key)
            continue

        # 新規登録ポップアップ
        if event == '-add-':
            # popup.close()
            popup = sg.Window("視点登録", buildAdd(), finalize=True)
            popup["x"].update(str(app.x))
            popup["y"].update(str(app.y))
            popup["z"].update(str(app.z))
            popup["rx"].update(str(app.rx))
            popup["ry"].update(str(app.ry))
            popup["rz"].update(str(app.rz))
            continue
        
        if popupEvent == '-add-confirm-':
            newCam = Cam(
                "", 
                uuid.uuid4(),
                popupValues["_label_"],
                float(popupValues["x"]),
                float(popupValues["y"]),
                float(popupValues["z"]),
                float(popupValues["rx"]),
                float(popupValues["ry"]),
                float(popupValues["rz"])
            )
            app.addCam(newCam)
            popup.close()
            popup=None
            treeData = buildTree(app.cams)
            window["-tree-"].update(treeData)
            continue
        if popupEvent == '-add-cancel-':
            popup.close()
            popup=None
            continue
        
        # 編集ポップアップ            
        if event == '-edit-':
            if len(values["-tree-"]) == 0: continue 
            
            keyForPopup = values["-tree-"][0]
            cam = app.getCam(keyForPopup)
            # popup.close()
            popup = sg.Window("視点編集", buildEdit(), finalize=True)
            popup["_label_"].update(cam.label)
            popup["x"].update(str(app.x))
            popup["y"].update(str(app.y))
            popup["z"].update(str(app.z))
            popup["rx"].update(str(app.rx))
            popup["ry"].update(str(app.ry))
            popup["rz"].update(str(app.rz))
            continue
        if popupEvent == '-edit-confirm-':
            updatedCam = Cam(
                "", 
                keyForPopup,
                popupValues["_label_"],
                float(popupValues["x"]),
                float(popupValues["y"]),
                float(popupValues["z"]),
                float(popupValues["rx"]),
                float(popupValues["ry"]),
                float(popupValues["rz"])
            )
            app.updateCam(updatedCam)
            popup.close()
            popup=None
            treeData = buildTree(app.cams)
            window["-tree-"].update(treeData)
            continue
        if popupEvent == '-edit-cancel-':
            popup.close()
            popup=None
            continue  
        
        # 削除ポップアップ
        if event == '-remove-':
            if len(values["-tree-"]) == 0: continue 
            
            keyForPopup = values["-tree-"][0]
            # popup.close()
            popup = sg.Window("確認", buildRemove(), finalize=True)
            continue
        if popupEvent == '-remove-confirm-':
            app.removeCam(keyForPopup)
            popup.close()
            popup=None
            treeData = buildTree(app.cams)
            window["-tree-"].update(treeData)
            continue
        if popupEvent == '-remove-cancel-':
            popup.close()
            popup=None
            continue
        
    window.close()