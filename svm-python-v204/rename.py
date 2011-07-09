#!/usr/bin/python
# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="aa755"
__date__ ="$Jul 8, 2011 6:54:07 PM$"

import commands;
from scipy.constants.constants import fine_structure
import re;

out=commands.getoutput('ls *.pcd');
files=out.split();

scene_map={};
#read the mapping file
mapfile=file('data_scenemapping.txt')
for line in mapfile:
    tokens=line.split(' ');
    scene_map[tokens[0]]=tokens[1].strip();
   # print tokens[0], "->",tokens[1]

    


#rename the files now
for file in files:
#    matches=re.search('([0-9]*)_(transformed_labeled_data_.*_segmented_xyzn\.pcd\.bag)\.pcd\.png',file)
    matches=re.search('([0-9]*)_(transformed_labeled_data.*_segmented_xyzn(_binary)?\.pcd)',file)
    frameNum=matches.group(1);
    imgfilename=matches.group(2);
    imflen=len(imgfilename);
    bagfilename=imgfilename#[0:imflen-12];
    newName='scene'+scene_map[bagfilename]+'_'+frameNum + file[len(file)-4:len(file)];
    #print file, "->",newName
    stat_out=commands.getstatusoutput( 'mv ' +file +' '+ newName);
    if(stat_out[0]!=0):
        print 'error: ',stat_out[1];
        exit(-1);
