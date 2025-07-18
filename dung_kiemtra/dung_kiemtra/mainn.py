#!/usr/bin/env python3
class KiemTraParent:
    def __init__(self, name):
        self.name = name

    def show_info_parent(self):
        print('day la bai kiem tra cuoi khoa:', self.name)

class KiemTraChild(KiemTraParent):
    def __init__(self, name, day):
        super().__init__(name)
        self.day=day

    def show_info_child(self):
        print('hom nay la:', self.day)
        self.show_info_parent()

kiemtra =KiemTraChild('Module A', 'thu tu')
kiemtra.show_info_child()