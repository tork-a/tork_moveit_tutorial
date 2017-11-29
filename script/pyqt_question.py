#!/usr/bin/env python

import sys
from PyQt4 import QtGui

def question_yn( qmsg='Message', title='Question' ):
    
    msgbox = QtGui.QMessageBox()
    result = msgbox.question( msgbox, title, qmsg, msgbox.Yes | msgbox.No, msgbox.No )
    
    if result == msgbox.Yes:
        return True
    else:
        return False


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)    

    print( question_yn() )
    print( question_yn("No/Yes") )
    print( question_yn("Question", "ROS Question") )
    
