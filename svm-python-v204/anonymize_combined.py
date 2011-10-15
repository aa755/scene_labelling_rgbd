#!/usr/bin/python
# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="aa755"
__date__ ="$Jul 8, 2011 6:54:07 PM$"

import commands;
import re;


scene_map={};
#read the mapping file
mapfile=file('data_scenemapping.txt')
for line in mapfile:
    tokens=line.split(' ');
    matches=re.search('(transformed_labeled_)(.*)(_segmented_xyzn(_binary)?\.pcd)',tokens[0])
    bagFileName=matches.group(2)+'.bag.stitched.bag'
    file=commands.getoutput('ls *'+matches.group(2)+'.*');
    newName='scene'+tokens[1].strip() + file[len(file)-4:len(file)];
    print file, "->",newName
    stat_out=commands.getstatusoutput( 'cp ' +file +' '+ newName);
    if(stat_out[0]!=0):
        print 'error: ',stat_out[1];
        exit(-1);

    

