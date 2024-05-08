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
            sg.Tree(key="-tree-", data=sg.TreeData(), headings=buildTreeHeadings(), enable_events=True, show_expanded=True, col0_width=24, auto_size_columns=False, col_widths=[6, 6, 6])
        ], 
        [
            sg.Button(button_text="Add", key="-add-"), sg.Button(button_text="Edit", key="-edit-"), sg.Button(button_text="Remove", key="-remove-")
        ]
    ]
    
    return contents

def posTable(readonly: bool):
    table = [
        sg.Column([[sg.Text()], [sg.Text(text="Eye", size=(8, 1))], [sg.Text(text="Focus", size=(8, 1))], [sg.Text(text="Up", size=(8, 1))]]),
        sg.Column([[sg.Text(text="X")], [sg.InputText(key="x", size=(6, 1), readonly=readonly)], [sg.InputText(key="fx", size=(6, 1), readonly=readonly)], [sg.InputText(key="ux", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y", size=(6, 1), readonly=readonly)], [sg.InputText(key="fy", size=(6, 1), readonly=readonly)], [sg.InputText(key="uy", size=(6, 1), readonly=readonly)]]),
        sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z", size=(6, 1), readonly=readonly)], [sg.InputText(key="fz", size=(6, 1), readonly=readonly)], [sg.InputText(key="uz", size=(6, 1), readonly=readonly)]])
    ], 
    return table

def buildTreeHeadings():
    return ["X", "Y", "Z"]

def buildTree(cams: "list[Cam]"):
    treeData = sg.TreeData()
    for cam in cams:
        treeData.insert(cam.parentKey, cam.key, cam.label, [str(cam.cp.eye.point.x), str(cam.cp.eye.point.y), str(cam.cp.eye.point.z)])
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
    def __init__(self, parentKey: str, key: str, label: str, cp: CameraPlacement, isLabel = False):
        super().__init__()
        self.__dict__ = self
        self.parentKey = parentKey
        self.key = key
        self.label = label
        self.isLabel = isLabel
        
        self.cp = cp

class CameraManager:
    def __init__(self):
        # 状態管理
        self.cams: list[Cam] = []
        self.cp = CameraPlacement()
                
        # ROS管理
        rospy.init_node("camera_manager", anonymous = True)
        self.pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)
        self.sub = rospy.Subscriber("/rviz/current_camera_placement", CameraPlacement, self.updateCurrentCam)
        
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
        self.pub.publish(cam.cp)
        return cam
        
    def loadCams(self, cams: "list[Cam]"):
        self.cams = cams
        return self.cams
    
    def loadCamsFromLocal(self):
        cam_list = rospy.get_param("/cam_list")
        cams = []
        for _ in cam_list:
            cp = CameraPlacement()
            
            cp.interpolation_mode = _["camera_placement"]["interpolation_mode"]
            cp.target_frame = _["camera_placement"]["target_frame"]
            cp.time_from_start.secs = _["camera_placement"]["time_from_start"]["secs"]
            cp.time_from_start.nsecs = _["camera_placement"]["time_from_start"]["nsecs"]
            cp.eye.header.seq = _["camera_placement"]["eye"]["header"]["seq"]
            cp.eye.header.stamp.secs = _["camera_placement"]["eye"]["header"]["stamp"]["secs"]
            cp.eye.header.stamp.nsecs = _["camera_placement"]["eye"]["header"]["stamp"]["nsecs"]
            cp.eye.header.frame_id = _["camera_placement"]["eye"]["header"]["frame_id"]
            cp.eye.point.x = _["camera_placement"]["eye"]["point"]["x"]
            cp.eye.point.y = _["camera_placement"]["eye"]["point"]["y"]
            cp.eye.point.z = _["camera_placement"]["eye"]["point"]["z"]
            cp.focus.header.seq = _["camera_placement"]["focus"]["header"]["seq"]
            cp.focus.header.stamp.secs = _["camera_placement"]["focus"]["header"]["stamp"]["secs"]
            cp.focus.header.stamp.nsecs = _["camera_placement"]["focus"]["header"]["stamp"]["nsecs"]
            cp.focus.header.frame_id = _["camera_placement"]["focus"]["header"]["frame_id"]
            cp.focus.point.x = _["camera_placement"]["focus"]["point"]["x"]
            cp.focus.point.y = _["camera_placement"]["focus"]["point"]["y"]
            cp.focus.point.z = _["camera_placement"]["focus"]["point"]["z"]
            cp.up.header.seq = _["camera_placement"]["up"]["header"]["seq"]
            cp.up.header.stamp.secs = _["camera_placement"]["up"]["header"]["stamp"]["secs"]
            cp.up.header.stamp.nsecs = _["camera_placement"]["up"]["header"]["stamp"]["nsecs"]
            cp.up.header.frame_id = _["camera_placement"]["up"]["header"]["frame_id"]
            cp.up.vector.x = _["camera_placement"]["up"]["vector"]["x"]
            cp.up.vector.y = _["camera_placement"]["up"]["vector"]["y"]
            cp.up.vector.z = _["camera_placement"]["up"]["vector"]["z"]
            cp.mouse_interaction_mode = _["camera_placement"]["mouse_interaction_mode"]
            cp.interaction_disabled = _["camera_placement"]["interaction_disabled"]
            cp.allow_free_yaw_axis = _["camera_placement"]["allow_free_yaw_axis"]
            
            cam = Cam(
                _["parentKey"],
                _["key"],
                _["label"],
                cp,
                _["isLabel"]           
            )
            cams.append(cam)
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
                "isLabel": cam.isLabel,
                
                # CameraPlacementをそのまま保存
                "camera_placement": {
                    "interpolation_mode": cam.cp.interpolation_mode,
                    "target_frame": cam.cp.target_frame,
                    "time_from_start": {
                        "secs": cam.cp.time_from_start.secs,
                        "nsecs": cam.cp.time_from_start.nsecs
                    },
                    "eye": {
                        "header": {
                            "seq": cam.cp.eye.header.seq,
                            "stamp": {
                                "secs": cam.cp.eye.header.stamp.secs,
                                "nsecs": cam.cp.eye.header.stamp.nsecs
                            },
                            "frame_id": cam.cp.eye.header.frame_id
                        },
                        "point": {
                            "x": cam.cp.eye.point.x,
                            "y": cam.cp.eye.point.y,
                            "z": cam.cp.eye.point.z,
                        },
                    },
                    "focus": {
                        "header": {
                            "seq": cam.cp.focus.header.seq,
                            "stamp": {
                                "secs": cam.cp.focus.header.stamp.secs,
                                "nsecs": cam.cp.focus.header.stamp.nsecs
                            },
                            "frame_id": cam.cp.focus.header.frame_id
                        },
                        "point": {
                            "x": cam.cp.focus.point.x,
                            "y": cam.cp.focus.point.y,
                            "z": cam.cp.focus.point.z,
                        },
                    },
                    "up": {
                        "header": {
                            "seq": cam.cp.up.header.seq,
                            "stamp": {
                                "secs": cam.cp.up.header.stamp.secs,
                                "nsecs": cam.cp.up.header.stamp.nsecs
                            },
                            "frame_id": cam.cp.up.header.frame_id
                        },
                        "vector": {
                            "x": cam.cp.up.vector.x,
                            "y": cam.cp.up.vector.y,
                            "z": cam.cp.up.vector.z,
                        },
                    },
                    "mouse_interaction_mode": cam.cp.mouse_interaction_mode,
                    "interaction_disabled": cam.cp.interaction_disabled,
                    "allow_free_yaw_axis": cam.cp.allow_free_yaw_axis
                }
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
    
    def updateCurrentCam(self, cp: CameraPlacement):
        self.cp = cp
        
    def getLabels(self):
        labelCams = filter(lambda cam: cam.isLabel == True, self.cams)
        labelDict = [{ "key": cam.key, "label": cam.label } for cam in labelCams ]
        return labelDict
    
##########################
### UIとRviz連携 start ###
##########################

if __name__ == "__main__":
    app = CameraManager()
    
    contents = build()
    window = sg.Window("Rviz Camera Manager", contents, finalize=True)
    
    # # ツリーヘッダー消去
    # window['-tree-'].Widget['show'] = 'tree'
    
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

        window["x"].update(str(app.cp.eye.point.x))
        window["y"].update(str(app.cp.eye.point.y))
        window["z"].update(str(app.cp.eye.point.z))
        window["fx"].update(str(app.cp.focus.point.x))
        window["fy"].update(str(app.cp.focus.point.y))
        window["fz"].update(str(app.cp.focus.point.z))
        window["ux"].update(str(app.cp.up.vector.x))
        window["uy"].update(str(app.cp.up.vector.y))
        window["uz"].update(str(app.cp.up.vector.z))

        
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
            popup["x"].update(str(app.cp.eye.point.x))
            popup["y"].update(str(app.cp.eye.point.y))
            popup["z"].update(str(app.cp.eye.point.z))
            popup["fx"].update(str(app.cp.focus.point.x))
            popup["fy"].update(str(app.cp.focus.point.y))
            popup["fz"].update(str(app.cp.focus.point.z))
            popup["ux"].update(str(app.cp.up.vector.x))
            popup["uy"].update(str(app.cp.up.vector.y))
            popup["uz"].update(str(app.cp.up.vector.z))
            continue
        
        if popupEvent == '-add-confirm-':
            if validateForm(popupValues["x"], popupValues["y"], popupValues["z"], popupValues["rx"], popupValues["ry"], popupValues["rz"]) == False:
                sg.popup_error('無効な値が入力されています。')
                continue
            newCp = app.cp
            newCp.eye.point.x = float(popupValues["x"])
            newCp.eye.point.y = float(popupValues["y"])
            newCp.eye.point.z = float(popupValues["z"])
            newCp.focus.point.x = float(popupValues["fx"])
            newCp.focus.point.y = float(popupValues["fy"])
            newCp.focus.point.z = float(popupValues["fz"])
            newCp.up.vector.x = float(popupValues["ux"])
            newCp.up.vector.y = float(popupValues["uy"])
            newCp.up.vector.z = float(popupValues["uz"])
            newCam = Cam(
                "", # parentKeyは今後使うかも 
                uuid.uuid4().hex,
                popupValues["_label_"],
                newCp
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
            popup["x"].update(str(app.cp.eye.point.x))
            popup["y"].update(str(app.cp.eye.point.y))
            popup["z"].update(str(app.cp.eye.point.z))
            popup["fx"].update(str(app.cp.focus.point.x))
            popup["fy"].update(str(app.cp.focus.point.y))
            popup["fz"].update(str(app.cp.focus.point.z))
            popup["ux"].update(str(app.cp.up.vector.x))
            popup["uy"].update(str(app.cp.up.vector.y))
            popup["uz"].update(str(app.cp.up.vector.z))
            continue
        if popupEvent == '-edit-confirm-':
            if validateForm(popupValues["x"], popupValues["y"], popupValues["z"], popupValues["fx"], popupValues["fy"], popupValues["fz"], popupValues["ux"], popupValues["uy"], popupValues["uz"]) == False:
                sg.popup_error('無効な値が入力されています。')
                continue
            
            updatedCp = app.cp
            updatedCp.eye.point.x = float(popupValues["x"])
            updatedCp.eye.point.y = float(popupValues["y"])
            updatedCp.eye.point.z = float(popupValues["z"])
            updatedCp.focus.point.x = float(popupValues["fx"])
            updatedCp.focus.point.y = float(popupValues["fy"])
            updatedCp.focus.point.z = float(popupValues["fz"])
            updatedCp.up.vector.x = float(popupValues["ux"])
            updatedCp.up.vector.y = float(popupValues["uy"])
            updatedCp.up.vector.z = float(popupValues["uz"])
                    
            updatedCam = Cam(
                "", # parentKeyは今後使うかも 
                keyForPopup,
                popupValues["_label_"],
                updatedCp
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