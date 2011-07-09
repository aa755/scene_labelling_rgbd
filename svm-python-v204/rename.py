#!/usr/bin/python
# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="aa755"
__date__ ="$Jul 8, 2011 6:54:07 PM$"

import commands;
import re;

out=commands.getoutput('ls *.png');
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
    matches=re.search('([0-9]*)_(transformed_labeled_data.*_segmented_xyzn.*)',file)
    frameNum=matches.group(1);
    imgfilename=matches.group(2);
    imflen=len(imgfilename);
    bagfilename=imgfilename[0:imflen-12];
    newName='scene'+scene_map[bagfilename]+'_'+frameNum
    print file, "->",newName
