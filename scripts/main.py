#!/usr/bin/env python

import math
import PySimpleGUI as sg
import rospy
import tf.transformations
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
import uuid
import tf
import yaml

############################
### ユーティリティ start ###
############################

# https://qiita.com/kitasenjudesign/items/e785e00736161ec238ae
def chokkou(r,theta,phi):
  x = r * math.sin(theta) * math.cos(phi)
  y = r * math.sin(theta) * math.sin(phi)
  z = r * math.cos(theta)
  return (x,y,z)

def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

# 条件に合致する最初の要素のインデックスを見つける
def find_first_index(lst, key, value):
    for index, item in enumerate(lst):
        if item[key] == value:
            return index
    return None 

################################
### PySimpleGUI UI定義 start ###
################################

def build():
    contents = [
        posTable(readonly=True), 
        [
            sg.Tree(key="-tree-", data=sg.TreeData(), headings=[], enable_events=True, show_expanded=True, col0_width=34)
        ], 
        [
            sg.Button(button_text="Add", key="-add-"), sg.Button(button_text="Edit", key="-edit-"), sg.Button(button_text="Remove", key="-remove-")
        ]
    ]
    
    return contents

def posTable(readonly: bool):
    table = [
        sg.Column([[sg.Text()], [sg.Text(text="Position", size=(8, 1))], [sg.Text(text="Rotation", size=(8, 1))]]),
        sg.Column([[sg.Text(text="X")], [sg.InputText(key="x", size=(6, 1), readonly=True)], [sg.InputText(key="rx", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y", size=(6, 1), readonly=True)], [sg.InputText(key="ry", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z", size=(6, 1), readonly=True)], [sg.InputText(key="rz", size=(6, 1), readonly=readonly)]])
    ], 
    return table

def buildTree(cams: "list[Cam]"):
    treeData = sg.TreeData()
    for cam in cams:
        treeData.insert(cam.parentKey, cam.key, cam.label, [])
    return treeData

def buildAdd():
    contents = [
        posTable(readonly=False),
        [
            sg.Text(text="ラベル"), sg.InputText(key="_label_", size=(15, 1))  
        ],
        [
            sg.Button(button_text="Confirm", key="-add-confirm-"), sg.Button(button_text="Cancel", key="-add-cancel-")
        ]
    ]
    return contents

def buildEdit():
    contents = [
        posTable(readonly=False),
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

def validateForm(*args):
    # 今のところはPoseパラメータをFloat型に変換できるかのバリデーションのみ
    for arg in args:
        try:
            float(arg)
            continue
        except:
            return False
    return True

######################
### Rviz連携 start ###
######################

class Cam(dict):
    def __init__(self, parentKey: str, key: str, label: str, pose: Pose, isLabel = False):
        super().__init__()
        self.__dict__ = self
        self.parentKey = parentKey
        self.key = key
        self.label = label
        self.pose = pose
        
        self.isLabel = isLabel
        
    def toCameraPlacement(self):
        cp = CameraPlacement()
        
        frame_id = "camera_frame"
        
        p = self.pose.position
        cp.eye.point = p
        cp.eye.header.frame_id = frame_id
        
        forward = tf.transformations.quaternion_matrix([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[:3, 2]
        d = 1.0
        f = Point(p.x - forward[0] * d, p.y - forward[1] * d, p.z - forward[2] * d)
        cp.focus.point = f
        cp.focus.header.frame_id = frame_id
        
        up = tf.transformations.quaternion_matrix([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[:3, 1]
        cp.up.vector.x = up[0]
        cp.up.vector.y = up[1]
        cp.up.vector.z = up[2]
        cp.up.header.frame_id = frame_id
        
        return cp

class CameraManager:
    def __init__(self):
        # 状態管理
        self.cams: list[Cam] = []
        self.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 0.0))
        
        # ROS管理
        rospy.init_node("camera_manager", anonymous = True)
        self.pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)
        self.sub = rospy.Subscriber("/rviz/current_camera_pose", Pose, self.updateCurrentCam)

        # 設定管理
        self.isAutosave = True
        
    def getCam(self, camKey):
        cam = next(filter(lambda cam: cam.key == camKey, self.cams), None)
        return cam
    
    def addCam(self, cam: Cam):
        self.cams.append(cam)
        
        self.autosave()
        return cam
    
    def updateCam(self, cam: Cam):        
        index = find_first_index(self.cams, "key", cam.key)
        self.cams[index] = cam
        
        self.autosave()
        return self.cams[index]
    
    def removeCam(self, camKey):
        index = find_first_index(self.cams, "key", camKey)
        self.cams.pop(index)
        
        self.autosave()
        return
    
    def selectCam(self, camKey):
        cam = next(filter(lambda cam: cam.key == camKey, self.cams))
        if cam == None:
            print("Missing cam index.")
            return
        if cam.isLabel == True:
            return 
        self.pub.publish(cam.toCameraPlacement())
        return cam
        
    def loadCams(self, cams: "list[Cam]"):
        self.cams = cams
        return self.cams
    
    def loadCamsFromLocal(self):
        cam_list = rospy.get_param("/cam_list")
        
        cams = [Cam(
            _["parentKey"],
            _["key"],
            _["label"],
            Pose(
                Point(_["x"], _["y"], _["z"]),
                Quaternion(_["rx"], _["ry"], _["rz"], _["rw"])
            ),
            _["isLabel"]
            ) for _ in cam_list]
        self.cams = cams
        return self.cams
    
    def autosave(self):
        if self.isAutosave == True:
            self.SaveCams()
    
    def SaveCams(self):
        cam_list = [
            { 
                "parentKey" : cam.parentKey,
                "key": cam.key,
                "label": cam.label,
                "x": float(cam.pose.position.x), # numpy.float64 -> float
                "y": float(cam.pose.position.y),
                "z": float(cam.pose.position.z),
                "rx": float(cam.pose.orientation.x),
                "ry": float(cam.pose.orientation.y),
                "rz": float(cam.pose.orientation.z),
                "rw": float(cam.pose.orientation.w),
                "isLabel": cam.isLabel
            }
            for cam in self.cams
        ]
        # ROSPARAMにセット
        rospy.set_param("/cam_list", cam_list)
        # config/config.yamlに書き出し
        # TODO: 書き出し方法がわからん
        # with open('config/config.yaml') as f:
        #     yaml.dump(
        #         { "cam_list": cam_list },
        #         f,
        #         default_flow_style=False,
        #         allow_unicode=True,
        #         sort_keys=False
        #     )
    
    def updateCurrentCam(self, pose: Pose):
        self.pose = pose
        print(
            "[",
            self.pose.position.x,
            self.pose.position.y,
            self.pose.position.z,
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
            "]")
        
    def getLabels(self):
        labelCams = filter(lambda cam: cam.isLabel == True, self.cams)
        labelDict = [{ "key": cam.key, "label": cam.label } for cam in labelCams ]
        return labelDict
    
##########################
### UIとRviz連携 start ###
##########################

if __name__ == "__main__":
    app = CameraManager()
    
    contents =  build()
    window = sg.Window("Rviz Camera Manager", contents, finalize=True)
    
    # ツリーヘッダー消去
    window['-tree-'].Widget['show'] = 'tree'
    
    cams = app.loadCamsFromLocal()
    treeData = buildTree(cams)
    window["-tree-"].update(treeData)

    window["-tree-"].bind('<Double-1>', "double-click-")
    
    popup = None
    keyForPopup = ""
    
    while True:
        print("LOOP CHECK")
        event, values = window.read(timeout=100, timeout_key='-timeout-')    
        popupEvent, popupValues = popup.read(timeout=100, timeout_key='-timeout-popup-') if not popup == None else (None, None)

        window["x"].update(str(app.pose.position.x))
        window["y"].update(str(app.pose.position.y))
        window["z"].update(str(app.pose.position.z))
        euler = euler_to_quaternion(app.pose.orientation)
        window["rx"].update(str(math.degrees(euler.x)))
        window["ry"].update(str(math.degrees(euler.y)))
        window["rz"].update(str(math.degrees(euler.z)))

        
        if event == sg.WIN_CLOSED:
            print('exit')
            break
        
        # ツリー操作
        ## 選択中のCamのキーを格納
        if len(values["-tree-"]) == 0: 
            keyForPopup = ""
        else: 
            keyForPopup = values["-tree-"][0]
        
        # ダブルクリックでカメラプリセットへカメラ移動
        if event == '-tree-double-click-':
            # ツリー内のアイテム未選択時は反応しない
            if len(values["-tree-"]) == 0: continue 
            
            key = values["-tree-"][0]
            app.selectCam(key)
            continue
        
        ##
        ## 新規登録ポップアップ
        ##
        if event == '-add-':
            # popup.close()
            popup = sg.Window("視点登録", buildAdd(), finalize=True)
            popup["x"].update(str(app.pose.position.x))
            popup["y"].update(str(app.pose.position.y))
            popup["z"].update(str(app.pose.position.z))
            euler = euler_to_quaternion(app.pose.orientation)
            popup["rx"].update(str(math.degrees(euler.x)))
            popup["ry"].update(str(math.degrees(euler.y)))
            popup["rz"].update(str(math.degrees(euler.z)))
            continue
        
        if popupEvent == '-add-confirm-':
            if validateForm(popupValues["x"], popupValues["y"], popupValues["z"], popupValues["rx"], popupValues["ry"], popupValues["rz"]) == False:
                sg.popup_error('無効な値が入力されています。')
                continue
            newPose = Pose(
                Point(
                    float(popupValues["x"]),
                    float(popupValues["y"]),
                    float(popupValues["z"])
                ), euler_to_quaternion(
                    Vector3(
                        math.radians(float(popupValues["rx"])),
                        math.radians(float(popupValues["ry"])),
                        math.radians(float(popupValues["rz"]))
                    )
                ))
            newCam = Cam(
                "", 
                uuid.uuid4().hex,
                popupValues["_label_"],
                newPose
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
        
        ##
        ## 編集ポップアップ
        ##            
        if event == '-edit-':
            # ツリー内のアイテム未選択時は反応しない
            if len(values["-tree-"]) == 0: continue 
            
            cam = app.getCam(keyForPopup)
            # popup.close()
            popup = sg.Window("視点編集", buildEdit(), finalize=True)
            popup["_label_"].update(cam.label)
            popup["x"].update(str(app.pose.position.x))
            popup["y"].update(str(app.pose.position.y))
            popup["z"].update(str(app.pose.position.z))
            euler = euler_to_quaternion(app.pose.orientation)
            popup["rx"].update(str(math.degrees(euler.x)))
            popup["ry"].update(str(math.degrees(euler.y)))
            popup["rz"].update(str(math.degrees(euler.z)))
            continue
        if popupEvent == '-edit-confirm-':
            if validateForm(popupValues["x"], popupValues["y"], popupValues["z"], popupValues["rx"], popupValues["ry"], popupValues["rz"]) == False:
                sg.popup_error('無効な値が入力されています。')
                continue
            
            updatedPose = Pose(
                Point(
                    float(popupValues["x"]),
                    float(popupValues["y"]),
                    float(popupValues["z"])
                ), euler_to_quaternion(
                    Vector3(
                        math.radians(float(popupValues["rx"])),
                        math.radians(float(popupValues["ry"])),
                        math.radians(float(popupValues["rz"]))
                    )
                ))
            updatedCam = Cam(
                "", 
                keyForPopup,
                popupValues["_label_"],
                updatedPose
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
        
        ##
        ## 削除ポップアップ
        ##
        if event == '-remove-':
            # ツリー内のアイテム未選択時は反応しない
            if len(values["-tree-"]) == 0: continue 

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