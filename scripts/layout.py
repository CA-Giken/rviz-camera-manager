#!/usr/bin/env python

import PySimpleGUI as sg

contents=None

def build():
    global contents
    contents = [
        [
            sg.Column([[sg.Text()], [sg.Text(text="Position")], [sg.Text(text="Rotation")]]),
            sg.Column([[sg.Text(text="X")], [sg.InputText(key="x")], [sg.InputText(key="rx")]]),
            sg.Column([[sg.Text(text="Y")], [sg.InputText(key="y")], [sg.InputText(key="ry")]]),
            sg.Column([[sg.Text(text="Z")], [sg.InputText(key="z")], [sg.InputText(key="rz")]])
        ], 
        [
            sg.Tree()
        ], 
        [
            sg.Button(button_text="Add", key="add"), sg.Button(button_text="Edit", key="edit"), sg.Button(button_text="Remove", key="remove")
        ]
    ]
    
    return contents

def buildTree(cams: "list[Cam]"):
    treeData = sg.TreeData()
    return treeData