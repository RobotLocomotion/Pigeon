#!/bin/bash
 # echo of all files in a directory

for file in *.STL
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'.dae' -om vn 
done
